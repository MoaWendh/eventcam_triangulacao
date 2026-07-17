// Autor: Moacir Wendhausen
// Projeto: VORIS
// Data: 13/01/2026
// Função: Este header contém as declarações das classes e funções relacionadas ao controle de hardware associado ao GPIO e PWM da Jetson Orin Nano

#pragma once

#include <string>
#include <gpiod.h>
#include <atomic>
#include <thread>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <filesystem>
#include <fcntl.h>
#include <cstdint>
#include <stdexcept>

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "parametros.h"



// Struct para as linhas, pinos, da Jetson:
struct GPIO_Lines {
    struct gpiod_line *triggerEventCam;
    struct gpiod_line *triggerNormalCam;
    struct gpiod_line *piscaLed;
    struct gpiod_line *controlLaser;
    struct gpiod_line *controlMotor_Pulso;
    struct gpiod_line *controlMotor_Dir;
};



// Classe para gerenciar o controle da interface SPI:
class SPIDevice {
private:
    // Atributo SPI File Descriptor "spi_ffd":
    int spi_fd;
    
    uint32_t sck;
    uint8_t numBits;

    bool spi_open;

public: 
    // O cosntrutor já inicializa o statado da SPI com false, será apenas true depois de executar o método openSPI(). 
    SPIDevice(): spi_open(false) {}; 

    ~SPIDevice() {
        if (spi_open) 
            close(spi_fd);
    }

    // Método obrigatório para abrir e cofnirgurar o HW da interface SPI com parametros definidos em "parametros.h":
    bool inicializa(const std::string& device, uint8_t mode, uint32_t clock, uint8_t bits_per_word);

    // Método chamaod para fechar a SPI se spi_open=true:
    void finaliza();
    
    // Método genérico para efetuar a leitura de 1 word de 16 bits na SPI:
    uint16_t readData(uint16_t data);
    
};




// Classe para tratar do objeto Motor com encoder acoplado:
class Motor {
private:
    uint16_t position_encoder;
    SPIDevice *spi_device; // Ponteiro para a interface isolada

    // Alguns atributos:
    int pulso_por_revolucao; 
    int resolucao_encoder;         

public:
    // O construtor agora exige a injeção da dependência da SPI
    Motor(SPIDevice *spi_device);

    ~Motor() = default;
    
    // Metodo para configurar os paramentros usados para operar o motor de passo:
    void setParamsMotor(int step, int pls_por_rev, int res_encoder);
    
    // Efetua a leitura da posição angular do encoder:
    void lerEncoder();

    // Posiciona o motor em um ângulo absoluto predefinido antes de iniciar a operação
    void irParaPosicaoInicial(gpiod_line *linePulso, gpiod_line *lineDir, double angulo);
    
    // Reposiciona o motor em newPosition:
    // Sentido horário -> dir = 1 (true)
    // Sentido anti-horário -> dir = 0 (false)
    void darPasso(gpiod_line *linePulso, gpiod_line *lineDir, bool dir, int delay);

    // Seter para ajustar o numero de pulsos por revolução do motor:
    void setPulsoPorRevolucao(int pulso_por_rev){
        pulso_por_revolucao= pulso_por_rev;    
    }

    // Seter para ajustar a resolução do encoder:
    void setResolucaoEncoder(int resolucao){
        resolucao_encoder= resolucao;
    }

    // Método para acessar o atributo privado position_encoder:
    uint16_t getPositionEncoder() {
        return position_encoder;
    };
};




// Classe com métodos para configurar a Jetson Orin nano:
class configJetson {
private:
    struct gpiod_chip *chip_ptr_interno= nullptr;
    GPIO_Lines gpios_internas= {nullptr, nullptr, nullptr, nullptr, nullptr};
    
    // Identificador do controlador GPIO no JetPack 6
    const std::string chipIO;

    // Inicializa as linhas do GPIO da Jetson relativas aso pinos do barramento conector J12:
    struct LineJetson {
        const int line_PWM_A= 43;   // PH.00  - Pino 33 (Controle PWM laser)
        const int line_PWM_B= 41;   // PG.06  - Pino 32 (Controle strobo Led)

        const int line_IO_A= 112;  // PR.04  - Pino 11 (Controle Motor_01)
        const int line_IO_B= 122;  // PY.00  - Pino 13 (Controle Motor_02)
        const int line_IO_C= 144;  // PAC.06 - Pino 7  (Trigger camera de eventos)
        const int line_IO_D= 106;  // PQ.06  - Pino 31 (Trigger camera convencional)
        const int line_IO_E= 51;   // PI.00  - Pino 40 (Pisca Led)  
    };

    // Pinos fisicos do barramento  GPIO J12 da Jetson Orin nano:
    struct PinoJetson {
        const int header_pin_PWM_A= 33; // Controle laser (Também PWM)
        const int header_pin_PWM_B= 32; // Controle strobo Led (Também PWM)

        const int header_pin_IO_A= 11; // Controle Motor_01 
        const int header_pin_IO_B= 13; // Controle Motor_02   
        const int header_pin_IO_C= 7;  // Trigger camera de eventos   
        const int header_pin_IO_D= 31; // Trigger camera convencional
        const int header_pin_IO_E= 40; // Pisca Led 
    };

    LineJetson lines;
    PinoJetson pinos;

public: 
    // Construtor padrão
    configJetson()
        : 
            chipIO("gpiochip0") {}; 

    //Destrutor da classe:
    ~configJetson(){
        liberaGPIO_Jetson(this->chip_ptr_interno, this->gpios_internas);    
        std::cout << "Classe configJetson encerrada e hardware liberado." << std::endl;
    }

    // Este método é chamado para configurar o barramento GPIO da Jetson, apenas isso:
    GPIO_Lines configura_GPIO_Jetson(struct gpiod_chip **chip_ptr); 

    void liberaGPIO_Jetson(struct gpiod_chip *chip, GPIO_Lines gpios);

    // Métodos Get para capturar as informações dos pinos configurados e lines:
    LineJetson getLines() { 
        return lines; 
    }

    PinoJetson getPinos() { 
        return pinos; 
    }

};




// Classe relacionada ao contrle de estado de Leds ou Lasers:
class LightController {
private:
    // Preferencialmente usar uma variável atômica em vez de primitiva, pois variáveis 
    // atomicas são mais adequadas para utilização com trheads, garantem sincornismo. 
    std::atomic<bool> is_active;

public:
    // Construtor da classe, inicializa a variável atômica com um estado seguro (desligado)
    LightController() : is_active(false) {}
     
    ~LightController(){      
    };

    // 
    void setRunning(bool status) {
        is_active = status;
    }

    // 
    bool getRunning() const {
        return is_active.load(); // 
    };
};



// Classe para ocntrole do PWM da Jetson usando os recursos do próprio Kernel do Linux usando a 
// interface Sysfs (Virtual files Systems).
// Na Jetson cada gerador de PWM é uma unidade de hardware independente.
// Ela disponibiza quatro chips de PWM:
// 3280000.pwm - pwmchip0 - PWM0 - Geral/LCD
// 32a0000.pwm - pwmchip1 - PWM5 - Geral
// 32c0000.pwm - pwmchip2 - PWM1 - Pino 33 - 
// 32e0000.pwm - pwmchip3 - PWM7 - Pino 32 - Usado para controle potencia do laser
// É pwmchipx pode variar a numeração. Para saber executar o comadno: 
// $ sudo cat /sys/kernel/debug/pwm 
class PWM {
private: 
    int64_t period; // Em nano segundos.
    int64_t dutyCycle; // Em percentual.
    std::atomic<bool> active; 

    std::string fullPath_chip;
    std::string canal;
    std::string nome;


    // Este método privado é usado para setar os parâmetros nos respectivos arquivos: periodo, duty-cycle e enable:
    bool writeToFile(std::string file, std::string value) {
        std::ofstream fs(fullPath_chip + canal + file);
        if (fs.is_open()) {
            fs << value;
            fs.close();
            return true;
        }
        else
            return false;
    }

public:
    // Construtor com as devidas inicializações das variáveis menbros privadas:
    PWM(int64_t periodo_ns, int64_t dutycycle_perc, std::string pathToChannel, std::string canal);

    // Destrutor da classe, desabilita o PWM e libera o canal:
    ~PWM();      

     // Seta a variável membro "periodo_pwm", que define a frequencia de trabalho do PWM, padrão é 1kHz:
    bool setPeriodo(int64_t periodo_ns);
    
    // Seta a variável membro "DutyCicle_pwm", que define a frequencia de trabalho do PWM, padrão é 1kHz:
    bool setDutyCycle(int64_t dutycycle_perc);

    // INicializa o path do chip referente ao PWM:
    void setPathFileChip(std::string path) { 
        fullPath_chip= path; 
    }
   
    void setChannel(std::string channel){     
        canal= channel;    
    }

    // Metodo get para o duty-cycle:
    long getDutyCycle(){     
        return dutyCycle;    
    }

    bool getStatus(){
        return active;
    }

    // Inicializa o canal pwm:
    void inicializa_canal();

    // Seta a variável membro "active_pwm", usada para guardar o status de habilitação do pwm.
    // Também habilita a geração do sinal pwm: 
    bool enable() ;

    // Desabilita o PWM:
    bool disable() ;

};