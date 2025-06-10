/**
 * @file sintetizador_audio.c
 * @brief Projeto Sintetizador de Áudio para BitDogLab Raspberry Pi Pico W
 *
 * @author Filipe Alves de Sousa
 * @date Junho de 2025
 *
 * @description
 * Este projeto implementa um sintetizador de áudio na placa BitDogLab Raspberry Pi Pico W.
 * Ele permite a captura de áudio de 10 segundos a uma taxa de amostragem de 16 kHz,
 * diretamente de um microfone conectado. O áudio capturado é armazenado em um
 * buffer na memória RAM e pode ser reproduzido posteriormente através de um buzzer
 * utilizando Modulação por Largura de Pulso (PWM). Simultaneamente, um medidor
 * de nível (VU-meter) de 12 barras é renderizado em tempo real em um display OLED
 * SSD1306 (128x64 pixels), oferecendo feedback visual da intensidade do áudio.
 *
 * @hardware
 * Os seguintes componentes de hardware são utilizados e suas respectivas conexões:
 * • Placa BitDogLab (Microcontrolador RP2040)
 * • Microfone de Eletreto: Conectado ao pino ADC2 (GPIO28) para captura de áudio analógico.
 * • Buzzer Passivo: Conectado ao pino GPIO10, utilizado para reprodução de áudio via PWM.
 * • Botão de Gravação (Rotulado 'A' na BitDogLab): Conectado ao pino GPIO5, aciona o início da gravação.
 * • Botão de Reprodução (Rotulado 'B' na BitDogLab): Conectado ao pino GPIO6, aciona o início da reprodução.
 * • LED Vermelho (na BitDogLab): Conectado ao pino GPIO13, indica o estado de gravação.
 * • LED Verde (na BitDogLab): Conectado ao pino GPIO11, indica o estado de reprodução.
 * • Display OLED SSD1306: Conectado à interface I2C1, com SDA no GPIO14 e SCL no GPIO15.
 *
 * @funcionalidades
 * As principais funcionalidades do sintetizador são:
 * • Gravação de Áudio: Ao pressionar o Botão A, o sistema inicia a gravação de 10 segundos
 * de áudio. As amostras são capturadas em 12 bits e depois reduzidas para 8 bits
 * para armazenamento eficiente. Durante a gravação, o VU-meter no OLED é atualizado
 * dinamicamente, e o LED vermelho acende.
 * • Armazenamento em Buffer: As amostras de áudio são armazenadas em um buffer
 * circular na RAM, com capacidade para 160 KB.
 * • Reprodução de Áudio: Ao pressionar o Botão B (após uma gravação), o sistema
 * reproduz as amostras armazenadas via PWM a 16 kHz. O VU-meter no OLED
 * também é atualizado em tempo real durante a reprodução, e o LED verde acende.
 * • Temporizadores Repetitivos: O uso de temporizadores (repeating timers) garante
 * a amostragem e a reprodução de áudio não-bloqueantes, otimizando o uso do processador.
 * • Atualização Suave do Display: A atualização do display OLED é feita com um
 * buffer de quadro (frame-buffered) para evitar cintilações e proporcionar uma experiência visual mais agradável.
 */


#include <stdio.h>      // Biblioteca padrão de entrada e saída (para usar printf, por exemplo)
#include <string.h>     // Biblioteca para manipulação de strings e memória (para usar memset e memcpy)
#include "pico/stdlib.h" // Funções padrão da Raspberry Pi Pico (controle de GPIOs, tempo, etc.)
#include "hardware/adc.h" // Funções para usar o Conversor Analógico-Digital (ADC)
#include "hardware/pwm.h" // Funções para usar o PWM (Modulação por Largura de Pulso)
#include "hardware/i2c.h" // Funções para usar a comunicação I2C (para o display OLED)
#include "pico/time.h"    // Funções para temporização (timers e atrasos)
#include "ssd1306.h"      // Biblioteca específica para o display OLED SSD1306
#include "ssd1306_font.h" // Biblioteca para as fontes de texto do display OLED

 // --- Configurações de Áudio ---
 // Imagine que a Pico 'escuta' o microfone várias vezes por segundo.
 // Essa é a 'taxa de amostragem'. Quanto maior, mais detalhes de áudio ela captura.
#define SAMPLE_RATE_HZ   16000     // 16.000 amostras por segundo (16 kHz)

// O tempo que queremos gravar o áudio.
#define DURATION_SEC     10        // 10 segundos de gravação

// O número total de 'fotos' (amostras) do áudio que a Pico vai tirar.
// É a taxa de amostragem multiplicada pela duração.
#define NUM_SAMPLES      (SAMPLE_RATE_HZ * DURATION_SEC) // 16000 * 10 = 160.000 amostras

// O tempo em microssegundos (milionésimos de segundo) entre cada 'foto' do áudio.
// 1 segundo = 1.000.000 microssegundos.
// Para 16.000 amostras/segundo, o tempo entre elas é 1.000.000 / 16.000 = 62.5 us.
#define TICK_US          62        // Aproximadamente 62 microssegundos

// --- Definições dos Pinos da Raspberry Pi Pico W (GPIOs) ---
// Estes são os 'endereços' dos pinos físicos na sua placa que serão conectados aos componentes.
#define BTN_REC_PIN      5         // Pino para o botão de Gravar
#define BTN_PLAY_PIN     6         // Pino para o botão de Reproduzir
#define LED_R_PIN        13        // Pino para o LED Vermelho
#define LED_G_PIN        11        // Pino para o LED Verde
#define BUZZER_PIN       10        // Pino para o Buzzer (onde o áudio será reproduzido)
#define MIC_GPIO         28        // Pino do microfone (um pino analógico, GPIO28 = ADC2)
#define MIC_CH           2         // Canal ADC correspondente ao MIC_GPIO (ADC2)

// --- Configurações do Display OLED (SSD1306) ---
// Estes são os parâmetros do seu pequeno display.
#define OLED_W           128       // Largura do display em pixels
#define OLED_H           64        // Altura do display em pixels
#define I2C_PORT         i2c1      // Qual interface I2C da Pico usar (a Pico tem i2c0 e i2c1)
#define I2C_SDA_PIN      14        // Pino de Dados (SDA) para o I2C do OLED
#define I2C_SCL_PIN      15        // Pino de Clock (SCL) para o I2C do OLED
#define OLED_ADDR        0x3C      // Endereço I2C padrão da maioria dos displays SSD1306

// --- Configurações da Barra de Nível de Áudio no OLED ---
// Estas definições calculam como as barrinhas do visualizador de áudio serão mostradas.
#define BAR_COUNT        12        // Quantas barras serão exibidas no visualizador
#define BAR_W            5         // Largura de cada barra em pixels
#define BAR_GAP          5         // Espaço entre as barras em pixels
// Largura total da área ocupada pelas barras (12 barras * (5 largura + 5 espaço) - 5 último espaço)
#define BAR_AREA_W       ((BAR_W+BAR_GAP)*BAR_COUNT - BAR_GAP)
// Posição X inicial para centralizar as barras na tela do OLED
#define BAR_X0           ((OLED_W - BAR_AREA_W)/2)
#define BAR_H            64        // Altura máxima que uma barra pode atingir (altura total do OLED)
#define BAR_TOP          (OLED_H - BAR_H) // Posição Y inicial (topo) das barras (neste caso, 0)

// --- Variáveis Globais (acessíveis em todo o programa) ---

// 'oled' é como a nossa 'caneta' para desenhar no display.
static ssd1306_t oled;

// 'backbuf' é uma 'tela de rascunho' invisível.
// Desenha-se nela primeiro e depois copia-se para o display para evitar 'flicker' (cintilação).
// O tamanho é (largura * altura / 8) porque cada pixel usa 1 bit, e há 8 bits em um byte.
static uint8_t backbuf[OLED_W * OLED_H / 8];

// Define os possíveis estados do nosso programa (máquina de estados).
// 'enum' cria um tipo de dado com valores nomeados.
typedef enum {
    IDLE = 0, // O programa está parado, esperando
    REC,      // O programa está gravando
    PLAY      // O programa está reproduzindo
} state_t;

// A variável 'state' guarda o estado atual do programa.
// 'volatile' é crucial aqui! Diz ao compilador que essa variável pode ser alterada a
// qualquer momento por algo 'externo' (como uma interrupção de botão),
// então ele deve sempre ler o valor mais recente, não uma cópia em cache.
static volatile state_t state = IDLE;

// 'buffer' é o nosso 'caderno' onde as amostras de áudio serão guardadas.
// Cada 'uint8_t' pode armazenar um valor de 0 a 255.
static uint8_t buffer[NUM_SAMPLES];

// 'wr_i' (write index) e 'rd_i' (read index) são como marcadores de página.
// 'wr_i' indica onde a próxima amostra será gravada.
// 'rd_i' indica de onde a próxima amostra será lida para reprodução.
// São 'volatile' porque são alterados dentro das funções de temporizador (interrupções).
static volatile uint32_t wr_i = 0; // Índice de escrita
static volatile uint32_t rd_i = 0; // Índice de leitura

// Flags booleanas (verdadeiro/falso) para indicar se a gravação ou reprodução terminaram.
// Também 'volatile' por serem alteradas em interrupções.
static volatile bool rec_done = false;  // Verdadeiro se a gravação terminou
static volatile bool play_done = false; // Verdadeiro se a reprodução terminou

// Flag para indicar que um novo 'quadro' (conjunto de barras) está pronto para ser exibido no OLED.
// Isso otimiza as atualizações do display.
static volatile bool frame_ready = false;

// Variável para armazenar o tempo da última interrupção de botão.
// Usada para o 'debounce' (evitar múltiplos cliques de um único toque).
static volatile uint32_t last_irq = 0;
// Tempo mínimo (em microssegundos) que deve passar entre duas interrupções de botão para serem válidas.
#define DEBOUNCE_US 200000 // 200 milissegundos

// --- Funções Auxiliares para o Display OLED ---
// 'static inline' sugere ao compilador para "colocar" o código dessas funções
// diretamente onde elas são chamadas, tornando-as mais rápidas para tarefas pequenas.

// Função para APAGAR uma barra específica no display OLED.
// 'idx' é o índice da barra (de 0 a 11).
static inline void clear_bar(uint8_t idx) {
    // Calcula a posição X inicial da barra.
    uint8_t x0 = BAR_X0 + idx * (BAR_W + BAR_GAP);
    // O display OLED é organizado em 'páginas' de 8 pixels de altura.
    // Este loop percorre as páginas que a barra ocupa.
    for (uint8_t page = BAR_TOP >> 3; page < 8; ++page)
        // 'memset' preenche uma área da memória do OLED com zeros, apagando a barra.
        memset(&oled.buffer[page * oled.width + x0], 0, BAR_W);
}

// Função para DESENHAR uma barra específica no display OLED com uma certa altura.
// 'idx' é o índice da barra, 'h' é a altura em pixels.
static inline void draw_bar(uint8_t idx, uint8_t h) {
    // Calcula a posição X inicial da barra.
    uint8_t x0 = BAR_X0 + idx * (BAR_W + BAR_GAP);
    // Loop para desenhar a barra pixel por pixel, de baixo para cima.
    for (uint8_t y = 0; y <= h; ++y) // Percorre a altura da barra
        for (uint8_t x = 0; x < BAR_W; ++x) // Percorre a largura da barra
            // 'ssd1306_draw_pixel' desenha um único pixel.
            // 'OLED_H-1 - y' faz a barra 'crescer' de baixo para cima.
            ssd1306_draw_pixel(&oled, x0 + x, OLED_H - 1 - y);
}

// --- Funções de Callback para Temporizadores Repetitivos ---
// Estas funções são chamadas automaticamente em intervalos de tempo fixos.

// Função de callback para o ADC (Gravação).
// É chamada a cada TICK_US durante a gravação.
static bool adc_cb(repeating_timer_t* rt) {
    // 1. Verifica se a gravação já completou o número de amostras.
    if (wr_i >= NUM_SAMPLES) {
        rec_done = true; // Marca a gravação como concluída
        return false;    // Retorna 'false' para parar o temporizador repetitivo
    }

    // 2. Lê o valor do ADC (microfone). O '>> 4' escala o valor de 12 bits (0-4095)
    // para 8 bits (0-255), que é o que guardamos no buffer.
    uint8_t v = adc_read() >> 4;
    buffer[wr_i++] = v; // Armazena a amostra no buffer e avança o índice de escrita

    // 3. Calcula a altura da barra para o display com base no valor da amostra.
    uint8_t h = (BAR_H - 1) * v / 255u; // Escala o valor (0-255) para a altura da barra (0-63)
    // Calcula o índice da barra atual (roda de 0 a BAR_COUNT-1, depois volta para 0)
    uint8_t idx = wr_i % BAR_COUNT;

    // 4. Atualiza o display: apaga a barra anterior e desenha a nova.
    clear_bar(idx);
    draw_bar(idx, h);

    // 5. Se a barra atual é a última do ciclo, significa que um 'quadro' completo
    // das barras foi atualizado, então marcamos 'frame_ready' para o display ser atualizado.
    if (idx == BAR_COUNT - 1)
        frame_ready = true;

    return true; // Retorna 'true' para continuar chamando o temporizador
}

// Função de callback para o PWM (Reprodução).
// É chamada a cada TICK_US durante a reprodução.
static bool pwm_cb(repeating_timer_t* rt) {
    // 1. Verifica se a reprodução já completou o número de amostras.
    if (rd_i >= NUM_SAMPLES) {
        play_done = true; // Marca a reprodução como concluída
        return false;     // Retorna 'false' para parar o temporizador
    }

    // 2. Lê o valor da amostra do buffer de áudio.
    uint8_t v = buffer[rd_i++]; // Pega a amostra e avança o índice de leitura

    // 3. Calcula a altura da barra para o display (igual ao adc_cb).
    uint8_t h = (BAR_H - 1) * v / 255u;
    // Calcula o índice da barra atual (igual ao adc_cb).
    uint8_t idx = rd_i % BAR_COUNT;

    // 4. Ajusta o 'duty cycle' (ciclo de trabalho) do PWM para o buzzer.
    // O duty cycle controla o 'volume' do som. Garantimos que não seja muito baixo nem muito alto.
    uint8_t duty = v;
    if (duty < 30) duty = 30;   // Valor mínimo para garantir que o buzzer faça algum som
    if (duty > 220) duty = 220; // Valor máximo para evitar distorção ou som muito alto
    pwm_set_gpio_level(BUZZER_PIN, duty); // Define o nível do PWM no pino do buzzer

    // 5. Atualiza o display: apaga a barra anterior e desenha a nova.
    clear_bar(idx);
    draw_bar(idx, h);

    // 6. Se a barra atual é a última do ciclo, marca 'frame_ready' para o display ser atualizado.
    if (idx == BAR_COUNT - 1)
        frame_ready = true;

    return true; // Retorna 'true' para continuar chamando o temporizador
}

// --- Função de Interrupção para os Botões (ISR) ---
// É chamada automaticamente quando um botão é pressionado (detecção de 'borda de descida').
// 'gpio' é o pino que gerou a interrupção.
// 'events' são os tipos de eventos (não usados diretamente aqui).
static void btn_isr(uint gpio, uint32_t events) {
    uint32_t now = time_us_32(); // Pega o tempo atual em microssegundos

    // Implementação do DEBOUNCE:
    // Se o tempo desde a última interrupção for menor que DEBOUNCE_US,
    // ignoramos essa interrupção (evita 'cliques' múltiplos por um único toque).
    if (now - last_irq < DEBOUNCE_US) return;
    last_irq = now; // Atualiza o tempo da última interrupção para o tempo atual

    // Define o estado do programa com base no botão que foi pressionado.
    // (gpio == BTN_REC_PIN) ? REC : PLAY é uma 'expressão condicional' (if-else em uma linha).
    // Se o pino for o de gravação, o estado é REC; senão, é PLAY.
    state = (gpio == BTN_REC_PIN) ? REC : PLAY;
}

// --- Funções de Inicialização ---

// Função para inicializar o display OLED.
static void init_oled(void) {
    // Inicializa a interface I2C com uma velocidade de 400 kHz.
    i2c_init(I2C_PORT, 400 * 1000);
    // Configura os pinos SDA e SCL para funcionarem como I2C.
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    // Ativa os resistores de 'pull-up' internos nos pinos I2C. Essencial para I2C.
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Inicializa a biblioteca do display OLED com os parâmetros definidos.
    ssd1306_init(&oled, OLED_W, OLED_H, OLED_ADDR, I2C_PORT);
    ssd1306_clear(&oled); // Limpa o display, apagando qualquer coisa na tela

    // Desenha mensagens iniciais no display.
    // ssd1306_draw_string(&oled, x, y, tamanho_fonte, "texto");
    ssd1306_draw_string(&oled, 8, 0, 1, "AUDIO SYNTH !");
    ssd1306_draw_string(&oled, 0, 10, 1, "PRESS A TO REC"); // Assumindo 'A' para REC
    ssd1306_draw_string(&oled, 0, 20, 1, "PRESS B TO PLAY"); // Assumindo 'B' para PLAY

    ssd1306_show(&oled); // Envia o conteúdo do buffer para ser exibido no display
}

// --- Funções de Lógica Principal ---

// Função para atualizar o display OLED se um novo 'quadro' de barras estiver pronto.
// É chamada repetidamente no loop principal para manter o display atualizado sem cintilação.
static inline void flush_if_ready(void) {
    if (!frame_ready) return; // Se não há um novo quadro pronto, sai da função.

    frame_ready = false; // Reseta a flag, pois o quadro será processado.

    // Copia o conteúdo do buffer 'visível' do OLED para o nosso 'buffer de rascunho' (backbuf).
    memcpy(backbuf, oled.buffer, sizeof(backbuf));

    // Salva o ponteiro original do buffer do OLED.
    uint8_t* orig = oled.buffer;
    // Redireciona o buffer do OLED para o nosso 'backbuf'.
    oled.buffer = backbuf;
    // Agora, ssd1306_show() vai enviar o conteúdo do 'backbuf' para o display.
    ssd1306_show(&oled);
    // Restaura o ponteiro do buffer do OLED para o original.
    // Assim, as próximas operações de desenho continuarão no buffer principal.
    oled.buffer = orig;
}

// Função para gerenciar o processo de gravação de áudio.
static void do_record(void) {
    printf("Gravando 10 segundos...\n"); // Mensagem para o terminal USB
    gpio_put(LED_R_PIN, 1);             // Acende o LED Vermelho para indicar gravação

    ssd1306_clear(&oled); // Limpa o display para mostrar o visualizador de barras
    ssd1306_draw_string(&oled, BAR_X0 - 10, 25, 1, "REC"); // Mensagem 'REC' no display

    // Inicializa o ADC e configura o pino do microfone.
    adc_init();
    adc_gpio_init(MIC_GPIO);
    adc_select_input(MIC_CH);

    // Reseta os índices e flags para uma nova gravação.
    wr_i = 0;
    rec_done = false;
    frame_ready = true; // Força uma primeira atualização do frame para limpar a tela visualmente.

    repeating_timer_t t; // Variável para o nosso temporizador

    // Adiciona um temporizador repetitivo que chamará 'adc_cb' a cada 'TICK_US' microssegundos.
    // O sinal de menos antes de TICK_US indica que a função deve ser chamada no momento exato,
    // não esperando pelo final da execução da função, o que é importante para amostragem precisa.
    add_repeating_timer_us(-TICK_US, adc_cb, NULL, &t);

    // Loop que espera a gravação terminar.
    // Enquanto a gravação não termina, ele continua atualizando o display.
    while (!rec_done) {
        flush_if_ready(); // Atualiza o display com as barras de áudio
    }

    cancel_repeating_timer(&t); // Para o temporizador quando a gravação termina
    gpio_put(LED_R_PIN, 0);     // Apaga o LED Vermelho
    printf("Gravacao concluida.\n"); // Mensagem para o terminal

    // Mensagens de conclusão no display.
    ssd1306_clear(&oled);
    ssd1306_draw_string(&oled, 8, 0, 1, "AUDIO SYNTH !");
    ssd1306_draw_string(&oled, 0, 10, 1, "REC DONE!");
    ssd1306_draw_string(&oled, 0, 20, 1, "PRESS B TO PLAY");
    ssd1306_show(&oled);
}

// Função para gerenciar o processo de reprodução de áudio.
static void do_play(void) {
    printf("Reproduzindo...\n"); // Mensagem para o terminal USB
    gpio_put(LED_G_PIN, 1);       // Acende o LED Verde para indicar reprodução

    ssd1306_clear(&oled); // Limpa o display para mostrar o visualizador de barras
    ssd1306_draw_string(&oled, BAR_X0 - 10, 25, 1, "PLAY"); // Mensagem 'PLAY' no display

    // Configura o pino do buzzer para funcionar com PWM (Modulação por Largura de Pulso).
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    // Obtém o 'slice' do PWM associado ao pino do buzzer.
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    // Obtém a configuração padrão do PWM.
    pwm_config cfg = pwm_get_default_config();
    // Define o divisor de clock do PWM. 1.0f para o clock total da Pico (125MHz).
    pwm_config_set_clkdiv(&cfg, 1.0f);
    // Define o valor máximo ('wrap') do contador do PWM. Isso define a resolução do PWM.
    // Com 255, o duty cycle pode variar de 0 a 255.
    pwm_config_set_wrap(&cfg, 255);
    // Inicializa o PWM com as configurações e o liga (true).
    pwm_init(slice, &cfg, true);

    // Reseta os índices e flags para uma nova reprodução.
    rd_i = 0;
    play_done = false;
    frame_ready = true; // Força uma primeira atualização do frame.

    repeating_timer_t t; // Variável para o nosso temporizador

    // Adiciona um temporizador repetitivo que chamará 'pwm_cb' a cada 'TICK_US' microssegundos.
    add_repeating_timer_us(-TICK_US, pwm_cb, NULL, &t);

    // Loop que espera a reprodução terminar.
    // Enquanto a reprodução não termina, ele continua atualizando o display.
    while (!play_done) {
        flush_if_ready(); // Atualiza o display com as barras de áudio
    }

    cancel_repeating_timer(&t); // Para o temporizador quando a reprodução termina
    gpio_put(LED_G_PIN, 0);     // Apaga o LED Verde
    // Desativa o PWM do buzzer para evitar ruídos residuais após a reprodução.
    pwm_set_gpio_level(BUZZER_PIN, 0); // Define o nível para 0
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_SIO); // Retorna o pino para função GPIO normal

    printf("Reproducao concluida.\n"); // Mensagem para o terminal

    // Mensagens de conclusão no display.
    ssd1306_clear(&oled);
    ssd1306_draw_string(&oled, 8, 0, 1, "AUDIO SYNTH !");
    ssd1306_draw_string(&oled, 0, 10, 1, "PLAY DONE!");
    ssd1306_draw_string(&oled, 0, 20, 1, "PRESS A TO REC");
    ssd1306_show(&oled);
}

// --- Função Principal: Onde o programa começa a rodar ---
int main(void) {
    // 1. Inicializa todas as bibliotecas padrão de I/O (entrada/saída).
    // Isso é necessário para que o 'printf' funcione e você possa ver as mensagens no terminal USB.
    stdio_init_all();

    // 2. Espera a conexão USB ser estabelecida.
    // O 'sleep_ms(100)' evita que a Pico consuma recursos excessivos enquanto espera.
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("Sistema inicializado!\n"); // Mensagem de que a Pico está pronta

    // 3. Inicializa o display OLED.
    init_oled();

    // 4. Configura os pinos dos botões:
    // BTN_REC_PIN (GPIO 5):
    gpio_init(BTN_REC_PIN);              // Inicializa o pino
    gpio_set_dir(BTN_REC_PIN, GPIO_IN);  // Configura como entrada
    gpio_pull_up(BTN_REC_PIN);           // Ativa o resistor de pull-up (mantém o pino HIGH quando não pressionado)
    // Configura uma interrupção para o botão de gravação.
    // Quando o botão é pressionado (borda de descida - quando o sinal vai de HIGH para LOW),
    // a função 'btn_isr' será chamada automaticamente.
    gpio_set_irq_enabled_with_callback(BTN_REC_PIN, GPIO_IRQ_EDGE_FALL, true, &btn_isr);

    // BTN_PLAY_PIN (GPIO 6):
    gpio_init(BTN_PLAY_PIN);             // Inicializa o pino
    gpio_set_dir(BTN_PLAY_PIN, GPIO_IN); // Configura como entrada
    gpio_pull_up(BTN_PLAY_PIN);          // Ativa o resistor de pull-up
    // Configura uma interrupção para o botão de reprodução.
    // É importante notar que a função 'btn_isr' já foi registrada como o callback
    // global para IRQs de GPIO. Então, basta habilitar a interrupção para este pino
    // e 'btn_isr' será chamada para ele também.
    gpio_set_irq_enabled(BTN_PLAY_PIN, GPIO_IRQ_EDGE_FALL, true);

    // 5. Configura os pinos dos LEDs como saída.
    gpio_init(LED_R_PIN);
    gpio_set_dir(LED_R_PIN, GPIO_OUT);
    gpio_init(LED_G_PIN);
    gpio_set_dir(LED_G_PIN, GPIO_OUT);

    printf("GP5 = REC | GP6 = PLAY (12x64 barras)\n"); // Mensagem de instrução no terminal

    // --- Loop Infinito do Programa ---
    // Este loop roda continuamente enquanto a Raspberry Pi Pico estiver ligada.
    while (true) {
        flush_if_ready(); // Sempre verifica e atualiza o display se houver um novo quadro pronto

        // Estrutura 'switch' para gerenciar os estados do programa.
        // O programa faz algo diferente dependendo do valor da variável 'state'.
        switch (state) {
        case REC:
            do_record();  // Chama a função para gravar áudio
            state = IDLE; // Após gravar, volta para o estado ocioso
            break;        // Sai do switch

        case PLAY:
            do_play();    // Chama a função para reproduzir áudio
            state = IDLE; // Após reproduzir, volta para o estado ocioso
            break;        // Sai do switch

        default:
            // Quando o estado é IDLE (ou qualquer outro não tratado acima),
            // 'tight_loop_contents()' é uma função otimizada para loops vazios,
            // que pode economizar energia do microcontrolador.
            tight_loop_contents();
            break;
        }
    }

    return 0; // O programa nunca deve chegar aqui, mas é boa prática ter um retorno.
}
