#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h> 
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "ssd1306.h"
#include "font.h"
#include "hardware/pwm.h"
#include "hardware/rtc.h"   //RTC
#include "pico/stdlib.h"
//Bibliotecas FatFS
#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_debug.h"
#include "rtc.h"
#include "sd_card.h"

#define TOTAL_AMOSTRAS 128            //Total de amostras por captura
#define BOTAO_CAPTURA 5               //Botão de captura
#define BOTAO_SD       6               //Botão para montar/desmontar SD
#define LED_RGB_R      13              //Led vermelho
#define LED_RGB_G      11              //Led verde
#define LED_RGB_B      12              //Led azul
#define BUZZER_PIN     21              //Buzzer

// Definição dos pinos I2C para o MPU6050
#define I2C_PORT i2c0                 // I2C0 usa pinos 0 e 1
#define I2C_SDA 0                     // Definição dos pinos I2C, Dados
#define I2C_SCL 1                     // Definição dos pinos I2C, Clock
static int addr = 0x68;               // Endereço I2C do MPU6050
// Definição dos pinos I2C para o display OLED
#define I2C_PORT_DISP i2c1            // I2C1 usa pinos 14 e 15
#define I2C_SDA_DISP 14               // Definição dos pinos I2C, Dados
#define I2C_SCL_DISP 15               // Definição dos pinos I2C, Clock
#define ENDERECO_DISP 0x3C            // Endereço I2C do display

uint buzzer_slice;  //Variável para armazenar o slice do buzzer
//Flags e variáveis globais do programa
volatile bool botao_captura_acionado = false;   
volatile bool botao_sd_acionado = false;
absolute_time_t ultimo_acionamento_captura;
absolute_time_t ultimo_acionamento_sd;
bool sd_montado = false;
volatile bool captura_em_andamento = false;
volatile bool cancelar_captura = false;
bool captura_inicializada = false;
int captura_indice = 0;

FIL file;       //Estrutura para armazenar os dados do arquivo
ssd1306_t ssd;  //Estrutura para armazenar os dados do display
static char filename[20] = "mpu6050.csv";   //Nome do arquivo CSV para armazenar os dados do MPU6050

//Funções para controlar os leds
void set_rgb_color(bool r, bool g, bool b){
    gpio_put(LED_RGB_R, r);
    gpio_put(LED_RGB_G, g);
    gpio_put(LED_RGB_B, b);
}
//Funções para controlar os leds de acordo com a cor (Facilitar na programação)
void led_amarelo()   { set_rgb_color(1, 1, 0); sleep_ms(500);} // R+G
void led_verde()     { set_rgb_color(0, 1, 0); }
void led_vermelho()  { set_rgb_color(1, 0, 0); }
void led_azul()      { set_rgb_color(0, 0, 1); }
void led_roxo()      { set_rgb_color(1, 0, 1); } // R+B
void led_branco()    { set_rgb_color(1, 1, 1); }    // Para desmontar o SD
void led_apagado()   { set_rgb_color(0, 0, 0); }

//Funções para piscar o led azul
void led_piscar_azul(){
    for (int i = 0; i < 6; i++){
        led_azul(); sleep_ms(100);
        led_apagado(0, 0, 0); sleep_ms(100);
    }
}

//Funções para piscar o led roxo
void led_piscar_roxo(){
    for (int i = 0; i < 6; i++){
        led_roxo(); sleep_ms(100);
        led_apagado(0, 0, 0); sleep_ms(100);
    }
}

//Funções para exibir mensagens no display
void oled_msg(const char* linha1, const char* linha2){
    ssd1306_fill(&ssd, false);
    //Bordas
    ssd1306_rect(&ssd, 0, 0, 128, 64, true, false);
    ssd1306_rect(&ssd, 1, 1, 128 - 2, 64 - 2, true, false);
    ssd1306_rect(&ssd, 2, 2, 128 - 4, 64 - 4, true, false);
    ssd1306_rect(&ssd, 3, 3, 128 - 6, 64 - 6, true, false);
    ssd1306_draw_string(&ssd, linha1, 15, 15);
    if(linha2)
        ssd1306_draw_string(&ssd, linha2, 20, 25);
    ssd1306_send_data(&ssd);
}

//Função para ler dados crus do acelerômetro, giroscópio
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp){
    uint8_t buffer[6];

    //Lê aceleração a partir do registrador 0x3B (6 bytes)
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++)    {
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    //Lê giroscópio a partir do registrador 0x43 (6 bytes)
    val = 0x43;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++){
        gyro[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    //Lê temperatura a partir do registrador 0x41 (2 bytes)
    val = 0x41;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 2, false);

    *temp = (buffer[0] << 8) | buffer[1];
}

//Função para definir estrutura do SD, acessar todos os dados, buffers, configurações, status e operações do cartão
static sd_card_t *sd_get_by_name(const char *const name){
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return sd_get_by_num(i);
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}
//Função para definir estrutura de sistema de arquivos FAT, realizar operações de leitura/gravação de arquivos (via f_open, f_write, f_read...)
static FATFS *sd_get_fs_by_name(const char *name){
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return &sd_get_by_num(i)->fatfs;
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}

//Funçao para o relogio do sistema, RTC
static void run_setrtc(){
    const char *dateStr = strtok(NULL, " ");
    if (!dateStr)
    {
        printf("Missing argument\n");
        return;
    }
    int date = atoi(dateStr);

    const char *monthStr = strtok(NULL, " ");
    if (!monthStr)
    {
        printf("Missing argument\n");
        return;
    }
    int month = atoi(monthStr);

    const char *yearStr = strtok(NULL, " ");
    if (!yearStr)
    {
        printf("Missing argument\n");
        return;
    }
    int year = atoi(yearStr) + 2000;

    const char *hourStr = strtok(NULL, " ");
    if (!hourStr)
    {
        printf("Missing argument\n");
        return;
    }
    int hour = atoi(hourStr);

    const char *minStr = strtok(NULL, " ");
    if (!minStr)
    {
        printf("Missing argument\n");
        return;
    }
    int min = atoi(minStr);

    const char *secStr = strtok(NULL, " ");
    if (!secStr)
    {
        printf("Missing argument\n");
        return;
    }
    int sec = atoi(secStr);

    datetime_t t = {
        .year = (int16_t)year,
        .month = (int8_t)month,
        .day = (int8_t)date,
        .dotw = 0, // 0 is Sunday
        .hour = (int8_t)hour,
        .min = (int8_t)min,
        .sec = (int8_t)sec};
    rtc_set_datetime(&t);
}

//Função para formatar o SD
static void run_format(){
    oled_msg("Formatando...", NULL);
    led_piscar_azul();
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        led_piscar_roxo();
        return;
    }
    led_piscar_azul();
    /* Format the drive with default parameters */
    FRESULT fr = f_mkfs(arg1, 0, 0, FF_MAX_SS * 2);
    if (FR_OK != fr){
        printf("f_mkfs error: %s (%d)\n", FRESULT_str(fr), fr);
        led_piscar_roxo();
    }
    led_verde();
}

//Função para montar o SD
static void run_mount(){
    led_amarelo();
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        led_piscar_roxo();
        return;
    }
    FRESULT fr = f_mount(p_fs, arg1, 1);
    if (FR_OK != fr){
        printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        led_piscar_roxo();
        return;
    }
    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = true;
    oled_msg("SD Montado!", NULL);
    led_verde();
    printf("Processo de montagem do SD ( %s ) concluído\n", pSD->pcName);
}

//Função para desmontar o SD
static void run_unmount(){
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        led_piscar_roxo();
        return;
    }
    FRESULT fr = f_unmount(arg1);
    if (FR_OK != fr){
        printf("f_unmount error: %s (%d)\n", FRESULT_str(fr), fr);
        led_piscar_roxo();
        return;
    }
    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = false;
    pSD->m_Status |= STA_NOINIT; // in case medium is removed
    led_branco();
    oled_msg("SD Desmontado", NULL);
    printf("SD ( %s ) desmontado\n", pSD->pcName);
}

//Função para obter o espaço livre do SD
static void run_getfree(){
    led_piscar_azul();
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    DWORD fre_clust, fre_sect, tot_sect;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs){
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        led_piscar_roxo();
        return;
    }
    FRESULT fr = f_getfree(arg1, &fre_clust, &p_fs);
    if (FR_OK != fr)
    {
        printf("f_getfree error: %s (%d)\n", FRESULT_str(fr), fr);
        led_piscar_roxo();
        return;
    }
    tot_sect = (p_fs->n_fatent - 2) * p_fs->csize;
    fre_sect = fre_clust * p_fs->csize;
    printf("%10lu KiB total drive space.\n%10lu KiB available.\n", tot_sect / 2, fre_sect / 2);
    oled_msg("Leitura feita", NULL);
    led_verde();
}

//Função para listar o conteúdo do SD
static void run_ls(){
    const char *arg1 = strtok(NULL, " ");
    if(!arg1)
        arg1 = "";
    char cwdbuf[FF_LFN_BUF] = {0};
    FRESULT fr;
    char const *p_dir;
    if (arg1[0]){
        p_dir = arg1;
    }else{
        fr = f_getcwd(cwdbuf, sizeof cwdbuf);
        if (FR_OK != fr){
            printf("f_getcwd error: %s (%d)\n", FRESULT_str(fr), fr);
            led_piscar_roxo();      //Liga led roxo se houver erro
            return;
        }
        p_dir = cwdbuf;
    }
    printf("Directory Listing: %s\n", p_dir);
    DIR dj;
    FILINFO fno;
    memset(&dj, 0, sizeof dj);
    memset(&fno, 0, sizeof fno);
    fr = f_findfirst(&dj, &fno, p_dir, "*");
    if (FR_OK != fr){
        printf("f_findfirst error: %s (%d)\n", FRESULT_str(fr), fr);
        led_piscar_roxo();
        return;
    }
    while(fr == FR_OK && fno.fname[0]){
        led_piscar_azul();
        const char *pcWritableFile = "writable file",
                   *pcReadOnlyFile = "read only file",
                   *pcDirectory = "directory";
        const char *pcAttrib;
        if (fno.fattrib & AM_DIR)
            pcAttrib = pcDirectory;
        else if (fno.fattrib & AM_RDO)
            pcAttrib = pcReadOnlyFile;
        else
            pcAttrib = pcWritableFile;
        printf("%s [%s] [size=%llu]\n", fno.fname, pcAttrib, fno.fsize);

        fr = f_findnext(&dj, &fno);
    }
    f_closedir(&dj);    //Fecha o diretorio
    oled_msg("Leitura feita", NULL);    //Enviar mensagem para o display
    led_verde();    //Ligar o led verde para indicar sucesso e SD pronto para uso
}

//Função para ler o conteúdo de um arquivo
static void run_cat(){
    char *arg1 = strtok(NULL, " ");
    if(!arg1){
        printf("Missing argument\n");
        return;
    }
    FIL fil;
    FRESULT fr = f_open(&fil, arg1, FA_READ);   //Abre o arquivo
    if (FR_OK != fr){   //Se nao conseguir abrir o arquivo
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr); //Imprime o erro
        led_piscar_roxo();  //Ligar o led roxo para indicar erro
        return;
    }
    char buf[256];
    while (f_gets(buf, sizeof buf, &fil)){   //Le o conteudo do arquivo
        led_piscar_azul();  //Ligar o led azul para indicar leitura
        printf("%s", buf);
    }
    fr = f_close(&fil); //Fecha o arquivo
    if(FR_OK != fr){   //Se nao conseguir fechar o arquivo
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr); //Imprime o erro
        led_piscar_roxo();  //Ligar o led roxo para indicar erro
    }
    oled_msg("Leitura feita", NULL);    //Envia mensagem para o display 
    led_verde();    //Liga o led verde e indica SD pronto para uso
}

//Função para ler o conteúdo de um arquivo e exibir no terminal
void read_file(const char *filename){
    FIL file;
    FRESULT res = f_open(&file, filename, FA_READ); //Abre o arquivo
    if (res != FR_OK){  //Se nao conseguir abrir o arquivo
        printf("[ERRO] Não foi possível abrir o arquivo para leitura. Verifique se o Cartão está montado ou se o arquivo existe.\n");
        led_piscar_roxo();  //Ligar o led roxo para indicar erro
        return;
    }
    char buffer[128];
    UINT br;
    printf("Conteúdo do arquivo %s:\n", filename);  //Imprime o cabeçalho
    while(f_read(&file, buffer, sizeof(buffer) - 1, &br) == FR_OK && br > 0){   //Enquanto nao chegar ao fim do arquivo
        led_piscar_azul();  //Ligar o led azul para indicar leitura em andamento
        buffer[br] = '\0';
        printf("%s", buffer);
    }
    f_close(&file);     //Fecha o arquivo
    printf("\nLeitura do arquivo %s concluída.\n\n", filename);  //Imprime a mensagem de conclusão
    oled_msg("Leitura feita", NULL);    //Imprime a mensagem no display
    led_verde();    //Ligar o led verde para indicar conclusão e SD pronto para uso
}

//Função para exibir os comandos disponíveis
static void run_help(){
    printf("\nComandos disponíveis:\n\n");
    printf("Digite 'a' para montar o cartão SD\n");
    printf("Digite 'b' para desmontar o cartão SD\n");
    printf("Digite 'c' para listar arquivos\n");
    printf("Digite 'd' para mostrar conteúdo do arquivo\n");
    printf("Digite 'e' para obter espaço livre no cartão SD\n");
    printf("Digite 'f' para iniciar/encerrar a captura de dados do IMU e salvar no arquivo\n");
    printf("Digite 'g' para formatar o cartão SD\n");
    printf("Digite 'h' para exibir os comandos disponíveis\n");
    printf("\nEscolha o comando:  ");
}

//Eestutura para armazenar os comandos disponíveis (Nome, função e ajuda, na ordem)
typedef void (*p_fn_t)();
typedef struct
{
    char const *const command;
    p_fn_t const function;
    char const *const help;
} cmd_def_t;

static cmd_def_t cmds[] = {
    {"setrtc", run_setrtc, "setrtc <DD> <MM> <YY> <hh> <mm> <ss>: Set Real Time Clock"},
    {"format", run_format, "format [<drive#:>]: Formata o cartão SD"},
    {"mount", run_mount, "mount [<drive#:>]: Monta o cartão SD"},
    {"unmount", run_unmount, "unmount <drive#:>: Desmonta o cartão SD"},
    {"getfree", run_getfree, "getfree [<drive#:>]: Espaço livre"},
    {"ls", run_ls, "ls: Lista arquivos"},
    {"cat", run_cat, "cat <filename>: Mostra conteúdo do arquivo"},
    {"help", run_help, "help: Mostra comandos disponíveis"}};

//Função para processar os comandos, interpretar o comando do serial e executar a função correspondente
static void process_stdio(int cRxedChar){
    static char cmd[256];
    static size_t ix;

    if (!isprint(cRxedChar) && !isspace(cRxedChar) && '\r' != cRxedChar &&
        '\b' != cRxedChar && cRxedChar != (char)127)
        return;
    printf("%c", cRxedChar); // echo
    stdio_flush();
    if (cRxedChar == '\r')
    {
        printf("%c", '\n');
        stdio_flush();

        if (!strnlen(cmd, sizeof cmd))
        {
            printf("> ");
            stdio_flush();
            return;
        }
        char *cmdn = strtok(cmd, " ");
        if (cmdn)
        {
            size_t i;
            for (i = 0; i < count_of(cmds); ++i)
            {
                if (0 == strcmp(cmds[i].command, cmdn))
                {
                    (*cmds[i].function)();
                    break;
                }
            }
            if (count_of(cmds) == i)
                printf("Command \"%s\" not found\n", cmdn);
        }
        ix = 0;
        memset(cmd, 0, sizeof cmd);
        printf("\n> ");
        stdio_flush();
    }
    else
    {
        if (cRxedChar == '\b' || cRxedChar == (char)127)
        {
            if (ix > 0)
            {
                ix--;
                cmd[ix] = '\0';
            }
        }
        else
        {
            if (ix < sizeof cmd - 1)
            {
                cmd[ix] = cRxedChar;
                ix++;
            }
        }
    }
}

//Função para resetar o MPU6050
static void mpu6050_reset(){
    // Dois bytes para reset: primeiro o registrador, segundo o dado (reset)
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
    sleep_ms(100); // Aguarda reset e estabilização

    // Sai do modo sleep (registrador 0x6B, valor 0x00)
    buf[1] = 0x00;
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
    sleep_ms(10); // Aguarda estabilização após acordar
}

//Função para lidar com as interrupções dos botoes, controlando flags de acionamento
void gpio_irq_handler(uint gpio, uint32_t events){
    absolute_time_t agora = get_absolute_time();

    //Controle do botão de captura
    if (gpio == BOTAO_CAPTURA){
        if (absolute_time_diff_us(ultimo_acionamento_captura, agora) > 300000){ //debounce 300ms
            botao_captura_acionado = true;      //flag de acionamento
            ultimo_acionamento_captura = agora; //armazena o tempo de acionamento
        }
    }
    //Controle do botão de montagem/desmontagem do SD
    if (gpio == BOTAO_SD){
        if (absolute_time_diff_us(ultimo_acionamento_sd, agora) > 300000){  //debounce 300ms
            botao_sd_acionado = true;           //flag de acionamento
            ultimo_acionamento_sd = agora;      //armazena o tempo de acionamento
        }
    }
}

//Função para alternar a montagem/desmontagem do cartão SD conforme o botão pressionado
void alternar_sd(){
    if(!sd_montado){    //Se o cartão SD nao estiver montado
        printf("[BOTÃO] Montando o cartão SD...\n");    //Imprime mensagem
        run_mount();    //Chama a funcao de montagem
        oled_msg("SD status:", "Montado!");     //Imprime mensagem no display
        sd_montado = true;  //flag de montagem
    }else{  //Se o cartão SD estiver montado
        printf("[BOTÃO] Desmontando o cartão SD...\n");     //Imprime mensagem
        run_unmount();  //Chama a funcao de desmontagem
        oled_msg("SD status:", "Desmontado!");  //Imprime mensagem no display
        sd_montado = false; //flag indicando desmontagem
    }
}

//Função para emitir beep n vezes do buzzer
void beep(int n){
    for (int i = 0; i < n; i++) {
        pwm_set_enabled(buzzer_slice, true);
        sleep_ms(100);
        pwm_set_enabled(buzzer_slice, false);
        sleep_ms(100);  // pausa entre beeps
    }
}

//Função para alternar a captura de dados do IMU
void alternar_captura(){
    if(!captura_em_andamento){  //Se nenhuma captura em andamento
        if(!sd_montado){        //Se o cartão SD nao estiver montado
            printf("[ERRO] Cartao SD nao montado.\n");  //Imprime erro
            oled_msg("Erro:", "Monte o SD!");           //Imprime erro no display
            return;
        }
        //Abre o arquivo para escrita
        FRESULT res = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
        if(res != FR_OK){   //Se nao conseguir abrir o arquivo
            printf("[ERRO] Falha ao abrir arquivo.\n");  //Imprime erro
            oled_msg("Erro:", "Arquivo!");
            led_piscar_roxo();  //Piscar led roxo indicando erro
            return;
        }

        UINT bw;    //Variavel para armazenar o numero de bytes escritos
        const char *header = "amostra,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n";  //Cabeçalho do arquivo
        f_write(&file, header, strlen(header), &bw);    //Escreve o cabeçalho
        //Inicia a captura, ativa as flags
        captura_em_andamento = true;    
        captura_inicializada = true;
        captura_indice = 0;
        cancelar_captura = false;
        beep(1);    //Beep para indicar inicio, como pedido na tarefa
        printf("[CAPTURA] Iniciada\n");   //Imprime que a captura iniciou
        oled_msg("Gravando...", NULL);    //Imprime que a captura iniciou no display
        led_vermelho();                   //Liga o led vermelho para indicar captura em andamento
    }else{
        beep(2);    //2 Beeps para indicar fim, como pedido na tarefa
        cancelar_captura = true;    //Ativa a flag de cancelamento
        printf("[CAPTURA] Cancelando...\n");    //Imprime que a captura foi cancelada
    }
}

//Função para inicializar os componentes de forma modular
void inicializar_componentes(){
    stdio_init_all();
    sleep_ms(5000);
    time_init();

    //Inicializa os botoes
    gpio_init(BOTAO_CAPTURA);
    gpio_init(BOTAO_SD);
    gpio_set_dir(BOTAO_CAPTURA, GPIO_IN);
    gpio_set_dir(BOTAO_SD, GPIO_IN);
    gpio_pull_up(BOTAO_CAPTURA);
    gpio_pull_up(BOTAO_SD);
    gpio_set_irq_enabled_with_callback(BOTAO_CAPTURA, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BOTAO_SD, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    //Inicializa o LED
    gpio_init(LED_RGB_R);
    gpio_init(LED_RGB_G);
    gpio_init(LED_RGB_B);
    gpio_set_dir(LED_RGB_R, GPIO_OUT);
    gpio_set_dir(LED_RGB_G, GPIO_OUT);
    gpio_set_dir(LED_RGB_B, GPIO_OUT);

    //Configura o buzzer com PWM
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    buzzer_slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_clkdiv(buzzer_slice, 125.0f);
    pwm_set_wrap(buzzer_slice, 999);
    pwm_set_gpio_level(BUZZER_PIN, 300);
    pwm_set_enabled(buzzer_slice, false);

    //Inicializa a I2C do Display OLED em 400kHz
    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);

    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO_DISP, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    //Inicialização da I2C do MPU6050
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    mpu6050_reset();    //Reseta o MPU6050
    led_amarelo();  //Liga o led amarelo para indicar inicio
}

int main(){
    inicializar_componentes();  //Inicializa os componentes

    printf("FatFS SPI example\n");
    printf("\033[2J\033[H"); // Limpa tela
    printf("\n> ");
    stdio_flush();

    led_branco();   //Liga o led branco para indicar que o cartao esta desmontado
    oled_msg("SD status:", "Nao montado");  //Imprime no display que o cartao nao esta montado
    run_help();     //Chama a funcao para exibir os comandos disponiveis
    while(true){
        if(botao_captura_acionado){ //Verifica se o botao de captura foi acionado
            alternar_captura();     //Chama a funcao para alternar a captura
            botao_captura_acionado = false; //Limpa a flag de botao acionado
        }

        if(captura_em_andamento && captura_inicializada){    //Verifica se a captura esta em andamento
            if(cancelar_captura || captura_indice >= TOTAL_AMOSTRAS){    //Verifica se a captura foi cancelada ou se o limite de amostras foi atingido
                f_close(&file); //Fecha o arquivo
                captura_em_andamento = false;
                captura_inicializada = false;

                if(cancelar_captura){    //Verifica se a captura foi cancelada
                    printf("[CAPTURA] Encerrada pelo usuario.\n");  //Imprime no terminal que a captura foi encerrada pelo usuario
                    oled_msg("Status:", "Finalizada");              //Imprime no display que a captura foi encerrada pelo usuario
                }else{
                    printf("[CAPTURA] Concluida.\n");               //Imprime no terminal que a captura foi concluida
                    oled_msg("Status:", "Concluida!");              //Imprime no display que a captura foi concluida
                }
                led_verde();    //Liga o led verde para indicar que a captura foi concluida e SD pronto para uso
            }else{                  //Se a captura nao foi cancelada, continua a captura
                int16_t acc[3], gyro[3], temp;  //Variaveis para armazenar os dados
                mpu6050_read_raw(acc, gyro, &temp); //Ler os dados
                float ax = acc[0] / 16384.0f;       //Converte os dados de acelerometro para o formato de ponto flutuante e normalizados
                float ay = acc[1] / 16384.0f;       
                float az = acc[2] / 16384.0f;
                float gx = gyro[0] / 131.0f;        //Converte os dados de giroscopio para o formato de ponto flutuante e normalizados
                float gy = gyro[1] / 131.0f;
                float gz = gyro[2] / 131.0f;

                char buffer[100];
                UINT bw;
                snprintf(buffer, sizeof(buffer), "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                         captura_indice + 1, ax, ay, az, gx, gy, gz);
                //Escreve os dados no arquivo
                FRESULT res = f_write(&file, buffer, strlen(buffer), &bw);
                if(res != FR_OK){       //Verifica se houve erro
                    printf("[ERRO] Falha na escrita.\n");    //Imprime erro
                    led_piscar_roxo();                      //Liga o led roxo para indicar erro
                    cancelar_captura = true;                //Cancela a captura
                }
                captura_indice++;   //Incrementa o indice
                sleep_ms(100);
            }
        }

        if(botao_sd_acionado){  //Verifica se o botao de SD foi acionado
            botao_sd_acionado = false;  //Limpa a flag de botao acionado
            alternar_sd();  //Chama a funcao para alternar o SD
            sleep_ms(1000);
            oled_msg("SD status:", sd_montado ? "Pronto!" : "Nao montado");  //Imprime no display o status do SD
        }

        int cRxedChar = getchar_timeout_us(0);
        if (PICO_ERROR_TIMEOUT != cRxedChar)
            process_stdio(cRxedChar);

        if (cRxedChar == 'a') // Monta o SD card se pressionar 'a'
        {
            printf("\nMontando o SD...\n");
            run_mount();
            printf("\nEscolha o comando (h = help):  ");
            sleep_ms(1000);
            oled_msg("SD status:", "Pronto!");
        }
        if (cRxedChar == 'b') // Desmonta o SD card se pressionar 'b'
        {
            printf("\nDesmontando o SD. Aguarde...\n");
            run_unmount();
            printf("\nEscolha o comando (h = help):  ");
        }
        if (cRxedChar == 'c') // Lista diretórios e os arquivos se pressionar 'c'
        {
            printf("\nListagem de arquivos no cartão SD.\n");
            run_ls();
            printf("\nListagem concluída.\n");
            printf("\nEscolha o comando (h = help):  ");
            sleep_ms(1000);
            oled_msg("SD status:", "Pronto!");
        }
        if (cRxedChar == 'd') // Exibe o conteúdo do arquivo se pressionar 'd'
        {
            read_file(filename);
            printf("Escolha o comando (h = help):  ");
            sleep_ms(1000);
            oled_msg("SD status:", "Pronto!");
        }
        if (cRxedChar == 'e') // Obtém o espaço livre no SD card se pressionar 'e'
        {
            printf("\nObtendo espaço livre no SD.\n\n");
            run_getfree();
            printf("\nEspaço livre obtido.\n");
            printf("\nEscolha o comando (h = help):  ");
            sleep_ms(1000);
            oled_msg("SD status:", "Pronto!");
        }
        if (cRxedChar == 'f') // Captura dados do IMU e salva no arquivo se pressionar 'f'
        {
            alternar_captura();
        }
        if (cRxedChar == 'g') // Formata o SD card se pressionar 'g'
        {
            printf("\nProcesso de formatação do SD iniciado. Aguarde...\n");
            run_format();
            printf("\nFormatação concluída.\n\n");
            printf("\nEscolha o comando (h = help):  ");
            sleep_ms(1000);
            oled_msg("SD status:", "Pronto!");
        }
        if (cRxedChar == 'h') // Exibe os comandos disponíveis se pressionar 'h'
        {
            run_help();
        }
        sleep_ms(500);
    }
    return 0;
}