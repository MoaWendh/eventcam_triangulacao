// ***************************************************************************************************************************************************************
// @ Autor: Moacir Wendhausen    
// @ Projeto: VORIS
// @ Data: 10/01/2026
//
// Classe principal do sistema de controle da triangulação com projeção de linha laser e camera de eventos. 
// 
// ***************************************************************************************************************************************************************


#include <gpiod.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <metavision/sdk/core/utils/cd_frame_generator.h>
#include <vector>

#include "Spinnaker.h"
#include "conv_camera.h"
#include "event_camera.h"
#include "control_hardware.h"
#include "parametros.h"



class ControlTriangulacao {
private:
    //Intsncia a struct que contém todos os parãmetros usados na triangulação, alterados em função do arquivo .json:
    PARAMETROS_GERAIS parametros_gerais;

    // Instancia os objetos que controlam as 2 PWMs da jetson Orin Nano:
    PWM pwm_A;
    PWM pwm_B;
    
    // Instancia o objeto que controla a configuração da Jetson Orina nano: 
    ConfigJetson configura_Jetson;
    // Instancia do objeto que controla o GPIO da Jetosn:
    GPIO_Lines gpios_actives;

    // Instancia a camera de eventos que é incializda pelo contrutor desta classe:
    ConvCamera *conv_cam_01;
    // Instancia que controla toa a interface SPI da Jetson:
    SPIDevice spi;

    // Onjeto que cntrola a luz led:
    LightController led;
    configFrameGenerator configFrameGen;
    Metavision::CDFrameGenerator *generator;
    

    // Struct ponteiro usada para acessar o chip de IO da jetson
    struct gpiod_chip *chip = nullptr;

        
    // Declara a câmera de eventos como um ponteiro, forma mais segura e mais facil de manipular:
    std::unique_ptr<EventCamera> event_cam;    


    // Declarações de todos os métodos privados da classe:
    int configuraGPIO_Jetson();   
    void pulseTrigger(int numPulse, int pin, int64_t duracaoPulso);

    bool ativaLed();
    void desativaLed();

    void incrementaPWM(PWM& pwm, bool useLed_LT2PR);
    void decrementaPWM(PWM& pwm);

    void ativaLinhaLaser(PWM &pwm);
    void desativaLinhaLaser(PWM &pwm);

    void varreduraTriangulacaoLaserPulsado();
    void varreduraTriangulacaoLaserContinuo();
      
    int configuraCameraDeEventos();
    bool detectaCamerasConectadas();
    void saveData_Mono_TriggerHW();
    void configuraFrameGenerator();
    void startTriggerSaveEventToFile(); 
    void stopTriggerSaveEventToFile();

    int configuraCameraConvencional();

    int lerParametrosTriangulacaoJson();

public:
    ControlTriangulacao() : 
        // Inicializando os objetos da classe PWM instanciados para controle:
        pwm_A(parametros_gerais.periodo_PWM_A, parametros_gerais.dutyCycle_PWM_A, parametros_gerais.channelToExport_A, "PWM A"),
        pwm_B(parametros_gerais.periodo_PWM_B, parametros_gerais.dutyCycle_PWM_B, parametros_gerais.channelToExport_B, "PWM B"),
        // Inicaliza o ponteiro da camera convencional com valor nulo:
        conv_cam_01(nullptr)
        {}

    ~ControlTriangulacao() { finalizaHardware(); }

    // Método que inicializa e configura todo o HW usado:
    int inicializaHardware(); 

    // Método que executa loop até a finalização do programa: 
    void executaLoopPrincipal(); 

    // Método para fechar portas e limpar memória:
    void finalizaHardware(); 
};