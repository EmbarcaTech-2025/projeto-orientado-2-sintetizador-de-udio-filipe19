# == NÃO EDITE AS LINHAS SEGUINTES para que a Extensão Raspberry Pi Pico VS Code funcione ==
# Este bloco é gerado e gerenciado pela extensão VS Code para Raspberry Pi Pico.
# Ele garante que o ambiente de build esteja corretamente configurado para a extensão.
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

# Define as versões do SDK, toolchain e picotool esperadas pela extensão.
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)

# Inclui o script de configuração da extensão se ele existir.
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

# --- Configuração Básica do CMake ---
# Define a versão mínima do CMake necessária para construir o projeto.
cmake_minimum_required(VERSION 3.13)

# Define o padrão da linguagem C e C++ a ser usado pelo compilador.
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Habilita a geração do 'compile_commands.json', útil para ferramentas de autocompletar e análise de código.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Configuração da Placa Raspberry Pi Pico ---
# Define o tipo de placa Pico a ser usado. 'pico_w' é para a versão com Wi-Fi.
# 'CACHE STRING' significa que essa variável pode ser configurada externamente uma vez e lembrada.
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Inclui o script do SDK que configura as variáveis de ambiente e caminhos para o SDK da Pico.
include(pico_sdk_import.cmake)

# --- Inicialização do Projeto ---
# Define o nome do projeto e as linguagens que serão utilizadas (C, C++, Assembly).
project(sintetizador_audio C CXX ASM)

# Inicializa o SDK da Pico para o projeto atual.
# Isso configura toolchains, flags de compilação, etc.
pico_sdk_init()

# --- Configuração do Executável Principal ---
# Adiciona o executável ao projeto.
# 'main.c' é o arquivo principal do seu código.
# 'ssd1306.c' é o arquivo fonte da biblioteca do display OLED que precisa ser compilado junto.
add_executable(${PROJECT_NAME}
    main.c
    ssd1306.c
)

# --- Configurações de Nomenclatura e Versão do Programa ---
# Define o nome do programa UF2 (o arquivo que você carrega na Pico).
pico_set_program_name(${PROJECT_NAME} "sintetizador_audio")
# Define a versão do programa.
pico_set_program_version(${PROJECT_NAME} "0.1")

# --- Configuração de Saída Serial (stdio) ---
# Habilita a saída serial via UART (Universal Asynchronous Receiver-Transmitter) para debug.
# O '0' indica que a UART0 é usada por padrão.
pico_enable_stdio_uart(${PROJECT_NAME} 0)
# Habilita a saída serial via USB (CDC ACM), o que permite ver 'printf' no terminal do PC.
# O '1' habilita essa funcionalidade.
pico_enable_stdio_usb(${PROJECT_NAME} 1)

# --- Bibliotecas Necessárias ---
# Linka as bibliotecas que o seu projeto utiliza.
# Essas bibliotecas contêm as funções como GPIO, ADC, PWM, I2C, etc.
target_link_libraries(${PROJECT_NAME}
    pico_stdlib    # Funções padrão da Pico (gpio_init, sleep_ms, etc.)
    hardware_i2c   # Suporte para comunicação I2C (para o OLED)
    hardware_adc   # Suporte para o Conversor Analógico-Digital (para o microfone)
    hardware_pwm   # Suporte para Modulação por Largura de Pulso (para o buzzer)
    # hardware_gpio  # Geralmente incluído com pico_stdlib, mas manter não faz mal.
                   # Seu código já usa gpio_init, gpio_set_dir, etc.
)

# --- Configurações de Diretórios de Inclusão (Headers) ---
# Informa ao compilador onde encontrar os arquivos de cabeçalho (.h) que o seu código inclui.
# 'PRIVATE' significa que esses includes são apenas para este projeto e não são expostos a outros que o linkem.
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}              # Onde está main.c, ssd1306.h, ssd1306_font.h
    ${CMAKE_CURRENT_LIST_DIR}/inc          # Se você tiver uma pasta 'inc' para seus headers
    # Caminhos para os headers do SDK (geralmente não são estritamente necessários aqui se pico_sdk_init() já os configurou bem)
    # ${PICO_SDK_PATH}/src/common/pico_stdlib/include
    # ${PICO_SDK_PATH}/src/rp2_common/hardware_adc/include
    # ${PICO_SDK_PATH}/src/rp2_common/hardware_pwm/include
    # ${PICO_SDK_PATH}/src/rp2_common/hardware_gpio/include
    # ${PICO_SDK_PATH}/src/rp2_common/hardware_i2c/include
)

# --- Geração de Arquivos Adicionais ---
# Gera o arquivo .uf2 (para upload via USB) e outros formatos (ELF, HEX) para o projeto.
pico_add_extra_outputs(${PROJECT_NAME})

# --- Configurações Específicas para o Display SSD1306 ---
# Define macros de pré-compilação para serem usadas no código C.
# Isso pode ser útil para configurar dimensões e endereço do OLED diretamente no build,
# mas no seu 'main.c' você já usa #define para isso, o que é igualmente válido.
# Se você definir aqui, pode remover os #define equivalentes no main.c,
# mas certifique-se de que a biblioteca SSD1306 os use corretamente.
target_compile_definitions(${PROJECT_NAME} PRIVATE
    SSD1306_WIDTH=128
    SSD1306_HEIGHT=64
    SSD1306_I2C_ADDR=0x3C
)

# --- Otimizações para Redução de Tamanho do Código e Eficiência ---
# Define opções de compilação para otimizar o código.
# '-Os': Otimiza para tamanho de código.
# '-fdata-sections', '-ffunction-sections': Coloca cada dado e função em sua própria seção,
# permitindo que o linker remova o que não for usado.
target_compile_options(${PROJECT_NAME} PRIVATE
    -Os
    -fdata-sections
    -ffunction-sections
)

# Define opções de linkagem.
# '-Wl,--gc-sections': Remove seções de código e dados não utilizadas, reduzindo o tamanho final do firmware.
target_link_options(${PROJECT_NAME} PRIVATE
    -Wl,--gc-sections
)