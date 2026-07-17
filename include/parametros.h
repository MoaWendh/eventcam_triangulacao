// Autor: Moacir Wendhausen
// Projeto: VORIS
// Data: 21/01/2026
// Função: Este header contém as declarações da struct "PARAMETROS_GERAIS", que é usada para armazenar os parâmetros gerais do sistema, 
// como os tempos de trigger, números de série das câmeras, configurações de PWM, entre outros.
// COntpem parâmetros associados tanto a ao IO da Jetson Orin Nano, IOs e PWMs, quanto a configuração da câmera de eventos, como os biases, 
// que são usados para configurar a câmera de eventos via SDK Metavision.

#pragma once


#include <cstdint>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <memory> 
#include <metavision/sdk/core/utils/cd_frame_generator.h>

#include <linux/spi/spidev.h>

struct PARAMETROS_GERAIS {
    // Parametros que definem as caracterísicas do triiger por HW da camera: 
    int64_t duracao_pulso_trigger   = 1000000; // em micro segundos
    int64_t duracao_pre_trigger     = 50000;   // em micro segundos
    int64_t duracao_pos_trigger     = 50000;   // em micro segundos
    //int64_t duracao_led             = 200000;  // em micro segundos
    int numero_ciclos_trigger       = 1;

    // Define se a gravação é feita em stereo ou mono, ou seja, se as duas câmeras de eventos são usadas ou apenas uma:
    const bool stereo= false;

    // Núemros de séries das cameras convencionais
    const std::string serialNumber_conv_cam_01= "25083333";
    const std::string serialNumber_conv_cam_02= "00000414"; 

    // Números de série das câmeras de eventos:
    const std::string serialNumber_event_cam0= "00000414"; // HD
    const std::string serialNumber_event_cam2= "00000679"; // VGA
    const std::string serialNumber_event_cam3= "00000680"; // VGA     
    
    // Definição dos paths referentes aos chips de IO da Jeson que geram o PWM: 
    const std::string channelToExport_A= "/sys/class/pwm/pwmchip3/";  
    const std::string channelToExport_B= "/sys/class/pwm/pwmchip2/"; 

    // Definição do Duty-cicle dos PWMs, valor em percentual.
    long dutyCycle_PWM_A    = 5;   //  PWM referente ao controle do duty cycle para blink led.
    //long dutyCycle_PWM_B    = 10;  // PWM referente ao controle da tensão.
    long dutyCycle_PWM_B    = 20;  // PWM referente ao controle da tensão.

    // Definição do periodo dos PWMs, que define a frquencia dos pulsos PWM.
    // ATENÇÂO!!!! valor definido em nano segundos.
    long periodo_PWM_A      = 10000000;  // A: referente ao blink do led (pino 32 da Jetson). 
    //long periodo_PWM_B      = 1000000;    // B: referente a tensão do led (pino 33 da Jetson).     
    long periodo_PWM_B      = 500000;    // B: referente a tensão do led (pino 33 da Jetson). 

    // Define se está usando o led de potencia LT2PR da Opto Engineering
    bool useLed_LT2PR       = false;

    // Declaração variaveis booleans do tipo atomic:
    bool useCamera_Conv     = false;
    bool useCamera_Event    = true;  
    
    // Flag que habilita menu:
    bool hab_exibe_menu     = false;
    
    // Definição dos parâmetros que guardam os valores máximos e mínimos dos biases da 
    // câmera de eventos, eles são usados para validar os dados lidos do arquivo JSON antes de serem gravados na câmera. 
    // Os biases max e min. são definidos em https://docs.prophesee.ai/stable/hw/manuals/biases.html 
    // Os calores para a ca~mera SilkyEvCam pertencem a geração Gen3.1 VGA, assim os valores máximos e mínimo
 
    const int bias_diff_default = 299; // Não alterar o valor do bias_diff, o default é 299.
    
    int bias_diff_on_min        = bias_diff_default + 75; // O valor mínimo do bias_diff_on é bias_dif_default + 75.
    int bias_diff_on_max        = bias_diff_default + 200; // O valor máximo do bias_diff_on é bias_dif_default + 200.
    
    int bias_diff_off_min       = 100; // O valor mínimo do bias_diff_off é 100.
    int bias_diff_off_max       = bias_diff_default - 65; // O valor máximo do bias_diff_off é bias_dif_default -65
    
    int bias_fo_min             = 1250;
    int bias_fo_max             = 1800;
    
    int bias_hpf_min            = 900;
    int bias_hpf_max            = 1800;
   
    int bias_refr_min           = 1300;
    int bias_refr_max           = 1800;

    // Relativo a interface SPI:
    uint8_t spi_mode        = SPI_MODE_1;      // Esta definido em <linux/spi/spidev.h>
    uint8_t spi_num_bits    = 8;               // Define quantos bits o controlador da SPI deve agrupar e processar por vez no seu shift register.
    uint32_t spi_sck        = 1000000;         // Define a frequencia, clock, do pino SPI-SCK
    std::string spi_device  = "/dev/spidev0.0"; // Define o device de HW do GPIO da Jetson que será utilizado para o canal SPI

    // Relativo aos parametros do motor e encoder:
    int motor_pulsos_por_revolucao = 3200; 
    int motor_res_encoder          = 16384;  // encoder com resolução de 14 bits.
    double motor_pos_ang_ini       = 0.0;    // Valor em graus daposição desejada.
    double motor_pos_ang_fim       = 0.0;    //
    int motor_num_ciclos           = 1;      // Numero de varreduras do motor. 
    int motor_delay_passo          = 200;     // valor em microsegundos.
    

    std::string pathFileParametersTriangulation = "../params_triangulacao.json"; 
    
};


//Valores de parâmetros para confiuração do frame genetator, eles são usados para configurar o gerador de frames da câmera de eventos, 
// que é responsável por criar os frames a partir dos eventos capturados pela câmera, e exibi-los em tempo real no viewer.
struct configFrameGenerator{ 
    std::mutex mutex;
    cv::Mat frame;
    cv::Mat canvas;
    cv::Mat canvas_events;
    cv::Mat canvas_menu;
    std::atomic<bool> showViewerEvents{true};
    std::string window_name= "Camera de Eventos - SilkyEvCam Gen3.1 VGA";
    int largura_canvas= 400;
    int janelaTemporalFrame= 50000; // Define o número de eventos que serão usados para gerar cada frame, ou seja, a quantidade de eventos que serão processados para criar um frame da câmera de eventos. Este parâmetro é crucial para equilibrar a fluidez da visualização e a quantidade de detalhes capturados em cada frame, especialmente em cenários com alta atividade de eventos.
    int freqCallback_frameGenerator= 30; // Define a frequência, em Hz, com que a callback do frame generator será chamada para atualizar os frames exibidos no dashboard. Este parâmetro é importante para garantir uma visualização fluida e responsiva dos frames gerados a partir dos eventos capturados pela câmera de eventos.
};
