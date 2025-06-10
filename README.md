
# Projetos de Sistemas Embarcados - EmbarcaTech 2025

Autor: **Filipe Alves de Sousa**

Curso: Residência Tecnológica em Sistemas Embarcados

Instituição: EmbarcaTech - HBr

Brasília-DF, junho de 2025

-----

# 🚀 Sintetizador de Áudio com BitDogLab (Raspberry Pi Pico W)


### 💻 Linguagem:
- **C** 


Este projeto visa explorar e implementar conceitos fundamentais de captura, armazenamento, processamento e reprodução de áudio digital utilizando a placa **BitDogLab Raspberry Pi Pico W**. Desenvolvido em **C estruturado no ambiente VS Code**, ele oferece uma plataforma didática para entender o funcionamento de conversores Analógico-Digitais (ADC), modulação de sinais (PWM) e otimização de memória.

## 🌟 Visão Geral do Projeto

O Sintetizador de Áudio permite que a BitDogLab grave um trecho de áudio de duração pré-definida através de um microfone, e posteriormente o reproduza utilizando um buzzer (ou um alto-falante externo com amplificador). A interatividade é garantida por botões físicos que controlam as funções de gravação e reprodução. Além disso, um display OLED e LEDs fornecem feedback visual em tempo real.

### 🎥 Demonstração

[link para um vídeo curto de demonstração do projeto funcionando no GitHub. Por exemplo: `https://youtu.be/seu-video-aqui`]

## ⚙️ Componentes de Hardware Utilizados

  * **Placa BitDogLab** (Baseada no microcontrolador RP2040)
  * **Microfone de Eletreto**: Conectado ao pino **ADC2 (GPIO28)**.
  * **Buzzer Passivo**: Conectado ao pino **GPIO10 (PWM)**.
  * **Botão 'A' (Gravação)**: Conectado ao pino **GPIO5**.
  * **Botão 'B' (Reprodução)**: Conectado ao pino **GPIO6**.
  * **LED Vermelho**: Conectado ao pino **GPIO13** (indicador de gravação).
  * **LED Verde**: Conectado ao pino **GPIO11** (indicador de reprodução).
  * **Display OLED SSD1306 (128x64 pixels)**: Conectado via **I2C1** (SDA no **GPIO14**, SCL no **GPIO15**).

## ✨ Funcionalidades Implementadas

  * **Aquisição de Sinal de Áudio**:
      * Captura de áudio do microfone a uma taxa de **16 kHz**.
      * Amostragem de 12 bits, com downsample para 8 bits para armazenamento eficiente.
  * **Armazenamento de Dados**:
      * Buffer em **RAM** com capacidade para 10 segundos de áudio (aproximadamente 160 KB).
      * Utilização de um buffer circular para gerenciamento de amostras.
  * **Síntese e Reprodução de Áudio**:
      * Reprodução das amostras armazenadas através de **PWM** no buzzer.
      * Frequência de reprodução ajustada à taxa de amostragem.
  * **Controle Interativo**:
      * **Botão 'A'**: Inicia o processo de gravação.
      * **Botão 'B'**: Inicia o processo de reprodução (após uma gravação).
  * **Feedback Visual**:
      * **LED Vermelho**: Acende durante a gravação.
      * **LED Verde**: Acende durante a reprodução.
      * **Display OLED**: Renderiza um **VU-meter de 12 barras** em tempo real, mostrando a intensidade do áudio durante a gravação e reprodução.
      * Atualização do display com *frame-buffering* para evitar cintilações.
  * **Otimização de Recursos**:
      * Uso de **temporizadores repetitivos (repeating timers)** para captura e reprodução de áudio não-bloqueantes.

## 💡 Conceitos Explorados

Este projeto abrange uma série de conceitos importantes em eletrônica, programação e processamento de sinais:

  * **Conversão Analógico-Digital (ADC)**: A base para transformar o áudio do mundo real em dados digitais que o microcontrolador pode entender.
  * **Taxa de Amostragem**: Crucial para a fidelidade do áudio, definindo quantas "fotos" do sinal são tiradas por segundo.
  * **Geração de Sinais (PWM)**: Demonstra como um sinal digital pode simular diferentes níveis de tensão analógica para gerar sons através do buzzer, controlando o *duty cycle*.
  * **Manipulação de Memória**: Otimização do uso da RAM limitada do microcontrolador para armazenar as amostras de áudio.
  * **Interrupções e Temporizadores**: Fundamentais para a execução de tarefas em tempo real sem "congelar" o programa principal.
  * **Processamento de Sinais Digitais (DSP)**: Mesmo que básico (normalização para display, ajuste de duty cycle), ilustra o potencial para filtragem, compressão e outros efeitos.
  * **Comunicação I2C**: Protocolo essencial para interagir com o display OLED.

## 🛠️ Etapas de Desenvolvimento (Guia Prático)

Este projeto foi construído seguindo as seguintes etapas, que podem servir como um guia para estudos futuros:

1.  **Aquisição do Sinal do Microfone**: Configurar o ADC para ler o microfone e visualizar as amostras.
2.  **Armazenamento dos Dados em Buffer**: Mover as amostras lidas do terminal para um array/buffer na RAM.
3.  **Configuração da Taxa de Amostragem**: Ajustar o tempo entre as leituras para garantir a fidelidade do áudio (e.g., 16 kHz).
4.  **Configuração do Período de Gravação**: Definir o tamanho do buffer com base na taxa de amostragem e duração desejada.
5.  **Manipulação do Sinal PWM**: Desenvolver a lógica para enviar os dados do buffer para o buzzer via PWM.
6.  **Reprodução Completa do Áudio**: Implementar a reprodução do áudio armazenado na frequência correta.
7.  **Controle com Botões**: Integrar os botões para iniciar e parar as funções de gravação e reprodução.
8.  **Feedback Visual com LEDs**: Adicionar LEDs para indicar os estados de gravação e reprodução.
9.  **Visualização da Forma de Onda no OLED**: Desenvolver a interface gráfica para o VU-meter.
10. **Aprimoramento da Saída de Áudio (Opcional)**: Conectar a um amplificador Classe-D para um alto-falante externo.
11. **Refinamento do Processamento de Áudio (Opcional)**: Implementar técnicas de filtragem de ruído via software.

## 📂 Estrutura do Projeto

```
.
sintetizador_audio/
├── inc/
│   ├── ssd1306.h             # Header da biblioteca do display OLED
│   ├── ssd1306_i2c.h
│   ├── ssd1306_i2c.c
│   └── ssd1306_font.h        # Header das fontes para o display OLED
│
├── main.c # Código principal do sintetizador
│   
└── CMakeLists.txt          # Script de build para o CMake
```




## 🚀 Como Compilar e Carregar

1.  **Pré-requisitos**: Certifique-se de ter o ambiente de desenvolvimento da Raspberry Pi Pico configurado (SDK, CMake, Make).
2.  **Clone o Repositório**:
    ```bash
    git clone https://github.com/seu-usuario/seu-repositorio.git
    cd seu-repositorio
    ```
3.  **Configurar Build**:
    ```bash
    mkdir build
    cd build
    cmake ..
    ```
4.  **Compilar**:
    ```bash
    make
    ```
5.  **Carregar para a Pico W**:
      * Após a compilação, um arquivo `.uf2` será gerado na pasta `build`.
      * Com a Pico W desconectada, pressione e segure o botão **BOOTSEL** e conecte o cabo USB ao computador.
      * A Pico W aparecerá como um dispositivo de armazenamento. Arraste e solte o arquivo `.uf2` para esta unidade.
      * A Pico W irá reiniciar e executar o programa.

-----

## 🤔 Reflexões Finais

  * Quais outras técnicas de programação poderiam ser usadas para aprimorar a gravação e reprodução do áudio (e.g., DMA, otimizações de buffer)?
  * Como seria possível gravar áudios mais extensos, considerando as limitações de memória da Pico (e.g., usando um cartão SD)?

-----
---

## 📜 Licença
GNU GPL-3.0.

