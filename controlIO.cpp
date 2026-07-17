#include "controlIO.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <gpiod.h>
#include <cmath>

#include "parametros.h"


// Método para efetuar a configuração do IO da Jetson:
GPIO_Lines configJetson::configura_GPIO_Jetson(struct gpiod_chip **chip_ptr) {
    GPIO_Lines lines_aux = {nullptr, nullptr, nullptr, nullptr, nullptr};

    //********************* Inicia configuração do barramenteo IO da Jetson*******************************************
    // Abre o chip definido na string "chipIO", membro privado, e armazena no ponteiro fornecido pelo main:
    *chip_ptr= gpiod_chip_open_by_name(chipIO.c_str());

    // Testa abertura do chip:
    if (!(*chip_ptr)) {
        perror("Erro ao abrir gpiochip0!!!");
        return lines_aux;
    }

    //*************** Captura os pinos, lines, no GPIO da Jetosn para todos os pinos que serão utilziados*************

    // Para o TRIGGER da camera de eventos usando lineF, pino fisico  7:
    lines_aux.triggerEventCam= gpiod_chip_get_line(*chip_ptr, lines.line_IO_C);
    if (!lines_aux.triggerEventCam) {
        std::cerr << "Erro: Nao foi possivel obter o pino para controle do trigger da camera de eventos na Linha: " << lines.line_IO_C << std::endl;
        return lines_aux;
    }
    gpiod_line_set_value(lines_aux.triggerEventCam, 0);

    // Para piscar o LED usando lineH, pino fisico 16:
    lines_aux.piscaLed= gpiod_chip_get_line(*chip_ptr, lines.line_IO_E);
    if (!lines_aux.piscaLed) {
        std::cerr << "Erro: Nao foi possivel obter o pino para controle do Led na linha: " << lines.line_IO_E << std::endl;
        return lines_aux;
    }
    gpiod_line_set_value(lines_aux.piscaLed, 0);    


    // Para o Trigger da câmera convencional, pino fisico 31:
    lines_aux.triggerNormalCam= gpiod_chip_get_line(*chip_ptr, lines.line_IO_D);
    if (!lines_aux.triggerNormalCam) {
        std::cerr << "Erro: Nao foi possivel obter o pino para controle do trigger da camera convencional na linha: " << lines.line_IO_D << std::endl;
        return lines_aux;
    }
    gpiod_line_set_value(lines_aux.triggerNormalCam, 0);    
    

     // Para o controle da fase 1 do motor, pino fisico 11:
    lines_aux.controlMotor_Pulso= gpiod_chip_get_line(*chip_ptr, lines.line_IO_A);
    if (!lines_aux.controlMotor_Pulso) {
        std::cerr << "Erro: Nao foi possivel obter o pino para controle de pulso do Motor na linha: " << lines.line_IO_A << std::endl;
        return lines_aux;
    }
    gpiod_line_set_value(lines_aux.controlMotor_Pulso, 0);  
    
     // Para o controle da fase 2 do motor, pino fisico 13:
    lines_aux.controlMotor_Dir= gpiod_chip_get_line(*chip_ptr, lines.line_IO_B);
    if (!lines_aux.controlMotor_Dir) {
        std::cerr << "Erro: Nao foi possivel obter o pino para controle da direcao do Motor na linha: " << lines.line_IO_B << std::endl;
        return lines_aux;
    }
    gpiod_line_set_value(lines_aux.controlMotor_Dir, 0);   
   

    // ************************** Faz um request das linhas de IO *********************************************************
    
    // Request do trigger da camera:
    if (gpiod_line_request_output(lines_aux.triggerEventCam, "triggerEventCam", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_aux.triggerEventCam, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // Usa o mapeamento físico privado da classe para o log:
        std::cout << "Jetson: trigger EvCam ................. pino: " << pinos.header_pin_IO_C << "  (Nivel= 0V)"<< std::endl;
    } 
    else {
        perror("[ERRO de request] Não foi possível configurar como OUTPUT o pino do trigger da camera de eventos.");
        return lines_aux;
    }

    // Request do controloe de piscagem do led:
    if (gpiod_line_request_output(lines_aux.piscaLed, "piscaLed", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_aux.piscaLed, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Jetson: Pisca Led ..................... pino: " << pinos.header_pin_IO_E << " (Nivel= 0V)" << std::endl;
    } 
    else {
        perror("[ERRO de Request] Não foi possível configurar como OUTPUT o pino do LED.");
        return lines_aux;
    }   


    // Request do controloe de trigger da camera convencional:
    if (gpiod_line_request_output(lines_aux.triggerNormalCam, "TriggerNormalCam", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_aux.triggerNormalCam, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Jetson: Trigger camera convencional ... pino: " << pinos.header_pin_IO_D << " (Nivel= 0V)"<< std::endl; 
    } 
    else {
        perror("[ERRO de Request] Não foi possível configurar como OUTPUT o pino de Trigger da camera convencional.");
        return lines_aux;
    }   


     // Request do controle 01 do Motor:
    if (gpiod_line_request_output(lines_aux.controlMotor_Pulso, "ControlMotor_Pulso", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_aux.controlMotor_Pulso, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Jetson: Controle MOTOR Pulso ......... pino: " << pinos.header_pin_IO_A << " (Nivel= 0V)"<< std::endl; 
    } 
    else {
        perror("[ERRO de Request] Não foi possível configurar como OUTPUT o pino de controle da Fase 01 do MOTOR.");
        return lines_aux;
    }    


     // Request do controle 02 do Motor:
    if (gpiod_line_request_output(lines_aux.controlMotor_Dir, "ControlMotor_Dir", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_aux.controlMotor_Dir, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Jetson: Controle MOTOR Dir ......... pino: " << pinos.header_pin_IO_B << " (Nivel= 0V)"<< std::endl; 
        std::cout << std::endl;
    } 
    else {
        perror("[ERRO de Request] Não foi possível configurar como OUTPUT o pino de controle da Fase 02 do MOTOR.");
        return lines_aux;
    }        


    // ANTES DO RETURN, salve o estado interno:
    this->chip_ptr_interno= *chip_ptr;
    this->gpios_internas= lines_aux;
    
    return lines_aux;
}




// Metodo que apeans fecha e libera o GPIO da Jetson:
void configJetson::liberaGPIO_Jetson(struct gpiod_chip *chip, GPIO_Lines gpios){

    // Libera as linhas individuais (se existirem)
    if (gpios.triggerEventCam) {
        gpiod_line_release(gpios.triggerEventCam);
        std::cout << "Jetson: Linha do trigger da camera de eventos liberada." << std::endl;
    }

    if (gpios.piscaLed) {
        gpiod_line_release(gpios.piscaLed);
        std::cout << "Jetson: Linha do LED liberada." << std::endl;
    }

     if (gpios.triggerNormalCam) {
        gpiod_line_release(gpios.triggerNormalCam);
        std::cout << "Jetson: Linha do trigger da camera convencional liberada." << std::endl;
    }

    if (gpios.controlLaser) {
        gpiod_line_release(gpios.controlLaser);
        std::cout << "Jetson: Linha do Laser liberada." << std::endl;
    }

     if (gpios.controlMotor_Pulso) {
        gpiod_line_release(gpios.controlMotor_Pulso);
        std::cout << "Jetson: Linha 1 do motor liberada." << std::endl;
    }   
 
      if (gpios.controlMotor_Dir) {
        gpiod_line_release(gpios.controlMotor_Dir);
        std::cout << "Jetson: Linha 2 do motor liberada." << std::endl;
    }     


    // Fecha o controlador (chip)
    if (chip) {
        gpiod_chip_close(chip);
        std::cout << "Jetson: Controlador " << chipIO << " fechado com sucesso." << std::endl;
    }
}



// Construtor da classe PWM, com as devidas inicializações das variáveis menbros privadas:
PWM::PWM(int64_t periodo_ns, int64_t dutyCycle_perc, std::string path, std::string n_canal) 
        : period(periodo_ns), 
          dutyCycle(dutyCycle_perc),
          fullPath_chip(path),           
          active(false), 
          canal("pwm0/"), 
          nome(n_canal) 
          {
                this->inicializa_canal();
                usleep(100000);
                this->setPeriodo(period);
                usleep(100000);
                this->setDutyCycle(dutyCycle);
                usleep(100000);
}

// Destrutor PWM:
PWM::~PWM() {
        disable();
        std::cout << "PWM " << canal.c_str() << " desabilitado e classe encerrada." << std::endl; 
 } 


// Método que inicializa o canal pwm:
void PWM::inicializa_canal() {
    // Exporta o canal:
    // Primeiro verifica se a pasta pwm0 existe se sim o canal já foi exportado.
    // A pasta pwm0 é criada com o export, se ela já existe não tem porque recriá-la, basta recofnigurar o pwm 
    if (!std::filesystem::exists(fullPath_chip + canal)){
        // Se não existe exporta:
        std::ofstream export_file(fullPath_chip + "export");
        export_file << "0"; 
        export_file.close(); 
    }
} 
    



// Ajusta o period do PWM:
bool PWM::setPeriodo(int64_t periodo_ns){
    // Atualiza  variável que guarda o periodo:
    bool set_ok= false;
    period= periodo_ns;

    // Configura o periodo:
    if (writeToFile("period", std::to_string(period))){
        std::cout << "Periodo PWM.............."<< this->nome << "= "<< period/1000000 << "ms ("<<1000000000/period << " Hz)" << std::endl;
        set_ok= true;
    }
    else
        std::cout << "[Erro] Não foi possível ajustar periodo canal:" << this->nome << std::endl;
    return set_ok;
}


// AJusta o Duty-Cycle do PWM:
bool PWM::setDutyCycle(int64_t dutyCycle_percentual) {
    dutyCycle= dutyCycle_percentual;
    bool set_ok= false;
    
    // Configura o duty_cycle
    int64_t dutyCycle_ns= (dutyCycle*period)/100;
    if (writeToFile("duty_cycle", std::to_string(dutyCycle_ns))){
        std::cout << "Duty-Cycle PWM..........."<< this->nome << "= "<< dutyCycle << "% ("<< dutyCycle_ns << "ns)." << std::endl;
        set_ok= true; 
    }
    else  
        std::cout << "[Erro] Não foi possível ajustar o duty-cycle do pwm canal:" << this->nome << std::endl;       
    return set_ok;            
}


// Habilita o PWM:
bool PWM::enable() { 
    if (writeToFile("enable", "1"))
        active= true;
    return active;   
}


// Desabilita o PWM:
bool PWM::disable() { 
    if (writeToFile("enable", "0"))
        active= false;
    return active;
}



// Método "openSPI" da classe "SPIDevice" usado para configurar os parametros lógicos do protocolo SPI:
// - O método "open()" Solicita ao kernel do Linux um canal de comunicação com o periférico associado àquele pino de Chip Select, ex: /dev/spidev1.0.
// - O comando "ioctl(SPI_IOC_WR_MODE, &mode)", instrui os registradores da controladora SPI da Jetson a operar no Modo 1 com CPOL= 0 e CPHA= 1, 
//   que é a exigência elétrica do encoder AS5048A.
// - O comando ioctl(SPI_IOC_WR_BITS_PER_WORD, &bits) configura o hardware da Jetson para fatiar e processar a serialização dos dados em blocos de 8 bits.
// - O comando ioctl(SPI_IOC_WR_MAX_SPEED_HZ, &speed) ajusta o divisor de frequência do gerador de clock interno, SCK, para pulsar exatamente a 
//   1 MHz durante as transmissões.
bool SPIDevice::inicializa(const std::string& device, uint8_t mode, uint32_t clock, uint8_t bits_per_word){
    sck= clock;
    numBits= bits_per_word;

    // Tenta abrir um canal de comunicação com o chip que ocntrola o SPI.
    // O SPI file descriptor "spi_fd" é o ID da SPI:
    spi_fd= open(device.c_str(), O_RDWR);

    // testa o atributo SPI File Descriptor "spi_ffd":
    if (spi_fd < 0) {
        // Caso algo de errado encerra com trwo para lançar um exceção, pois é um erro crítico. 
        throw std::runtime_error("Falha ao abrir o dispositivo SPI: " + device);
        return 0;
    }

    // Aplica as configurações lógica usando o método "ioctl()", definindo o SPI_SCK e o tamanho da palavra definida em numBits. 
    // Caso ocorram flahgas o edvice é fechado:
    if ((ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) == -1) || (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &numBits) == -1) || (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &sck)== -1)) {     
        close(spi_fd);
        throw std::runtime_error("Falha ao configurar os parâmetros da SPI.");
        return 0;
    }

    // Sinaliza o status da SPI que foi aberta e está ok: 
    spi_open= true;

    std::cout<< std::endl;
    std::cout<< "SPI aberta com os seguintes parametros:"<< std::endl;
    std::cout<< "Device SPI:.............." << device << std::endl;
    std::cout<< "SPI_SCK:................." << std::fixed << std::setprecision(2) << clock/1000000.0 << " MHz"<< std::endl;

    return spi_open;

}



// Método usado para fechar a interface SPI usando o ID spi_fd
void SPIDevice::finaliza(){
    if (spi_open){ 
        close(spi_fd);
        spi_open= false;
        std::cout << " Status spi_open= " << spi_open << std::endl;
    }    
    else
        std::cout << " SPI já estava fechada | spi_open= )" << spi_open << std::endl;;
}




// Metodo para escrita e leitura na SPI, escreve o array tx[] e recebe o array rx[]:
// Tamanho tx[]= 16 bits
// Tamanho rx[]= 16 bits
// rx[0] é o byte mais significativo.
uint16_t SPIDevice::readData(uint16_t data) {
    // Array de bytes tx[] para transmitir pela SPI pelo pino MOSI, inicilizado com "data" em 2 posições tx[0] e tx[1]:
    uint8_t tx[]= { static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF) };

    // Array de bytes rx[] para recepção dos valores lidos no encoder pelo pino MISO:
    // Declara com tamanho 2, para gaurdar 2 bytes. rx[0] e rx[1]:
    uint8_t rx[2]= {0};

    // Criando uma variável local chamada tr que é do tipo struct "spi_ioc_transfer":
    struct spi_ioc_transfer tr= {};

    // Preenchimento da estrutura da transação SPI
    tr.tx_buf= (unsigned long)tx;
    tr.rx_buf= (unsigned long)rx;
    tr.len= 2;
    tr.speed_hz= sck;
    tr.bits_per_word= numBits;

    // Chama o metodo "ioctl()", Input/Output Control, que é uma chamada de sistema, system call, para disparar a execução da 
    //transferência de dados no hardware. 
    // É ela que pega as regras que foram definidas na estrutura "tr" acima, e manda o kernel do Linux atuar nos pinos físicos da Jetson.
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        std::cerr << "Erro na transmissão SPI no file descriptor: " << spi_fd << std::endl;
        return 0;
    }

    return (rx[0] << 8) | rx[1];
}



// Construtor da classe ControlMotor:
Motor::Motor(SPIDevice* spi_int) 
: 
    position_encoder(0), 
    spi_device(spi_int), 
    pulso_por_revolucao(3200), 
    resolucao_encoder(16384) {};



// Método que efetua a leitura do encoder:
void Motor::lerEncoder() {
    // Apenas para segurnaça, testa se a instancia spi_device foi criada: 
    if (!spi_device) 
        return;

    // Primeira transferência para lidar com o atraso do pipeline do AS5048A.
    // O valor retornado aqui refere-se à transação anterior, logo é descartado:
    spi_device->readData(0xFFFF);

    // Segunda transferência captura a resposta real, dado bruto do encoder que contém,
    // dentre outras coias, a posição angular:
    uint16_t dado_bruto= spi_device->readData(0xFFFF);

    // Aplica a máscara 0x3FFF para extrair os 14 bits de posição angular.
    // descartando:
    // - bit 14: flag de erro 
    // - bit 15: paridade.
    // Posteriormente eles devem ser testados para aumentar a confibilidade da comunicação.
    position_encoder = dado_bruto & 0x3FFF;
}



// Metodo para posicionar o motor atuando diretamete no GPIO da Jetso Orin nano.
// O Motor é atuado diretamente pelo driver TB6600, cujos parametros são:
// 1- Largura mínima do pulso (Pulse Width): O chip exige que o pino STEP fique em nível ALTO por, no mínimo, 1.2 a 2.2 microssegundos, 
// dependendo da temperatura e do optoacoplador da placa. O nível BAIXO também precisa durar esse mesmo tempo mínimo.
// 2- Frequência Máxima: O circuito do driver suporta uma frequência máxima de trem de pulsos de 20 kHz a 40 kHz.
// 3- Setup Time de Direção: Quando o pino DIR sofre variação de direção, é necessário esperar cerca de 2 a 5 microssegundos antes de enviar o 
// primeiro pulso no pino STEP, se o pulso for enviado junto com a mudança de direção, o driver pode dar o primeiro passo para o lado errado.
void Motor::darPasso(gpiod_line *linePulso, gpiod_line *lineDir, bool dir, int delay) {
    // Define o sentido de giro, escrita em IO, em função do paramentro de entrada "dir":
    if (dir) {
        // Coloca o pino 13 em nivel alto, indicando sentido horário:
        gpiod_line_set_value(lineDir, 1);
    } else {
        // Coloca o pino 13 em nivel baixo, indicando sentido anti-horário:
        gpiod_line_set_value(lineDir, 0);
    }

    // Um pequeno delay após mudar a direção para garantir  que o driver do motor reconheceu o estado lógico antes do primeiro pulso:
    std::this_thread::sleep_for(std::chrono::microseconds(delay)); 

    // Coloca o pino 11 em nivel alto, rising edge do pulso no motor:
    gpiod_line_set_value(linePulso, 1); 

    // Delay para respeitar a temporização:
    std::this_thread::sleep_for(std::chrono::microseconds(delay));

    // Coloca o pino 11 em nivel baixo, falling edge do pulso no motor:
    gpiod_line_set_value(linePulso, 0);  

    // Delay para respeitar a temporização:
    std::this_thread::sleep_for(std::chrono::microseconds(delay));

}



// Determina a posição absoluta do motor a partir da posição angular definida no parametro de entrada "angulo":
void Motor::irParaPosicaoInicial(gpiod_line *line_to_pulso, gpiod_line *line_to_dir, double pos_angulo) {
    // Converte o "angulo" de entrada, em graus, para passos do encoder, valor entre 0 e 16383:
    int16_t pos_absoluto= static_cast<int16_t>((pos_angulo*resolucao_encoder)/360.0);
    
    // O loop while equivale a uma malha fechada, só finaliza quando atingir a condição de parada
    // definida em "if (std::abs(delta_pos) <= 3)":
    while (true){
         // Atualiza a leitura atual do encoder, verifica onde o encoder se encontra e o valor será gaurdado 
        // no atributo "position_encoder":
        lerEncoder();

        int16_t pos_atual= position_encoder;
        
        // Calcula a distancai entre as posições atual e a posição desejada:
        int32_t delta_pos= pos_absoluto - pos_atual;

        // Determina o caminho mais curto para reposicionar o motor:
        if (delta_pos> (resolucao_encoder/2)) {
            delta_pos -= resolucao_encoder;
        } 
        else if (delta_pos< -(resolucao_encoder/2)) {
            delta_pos += resolucao_encoder;
        }

        // Condição de parada!!!
        // Se já estiver na posição atual, considerando uma margem de histerese de ruído do encoder,
        // o loop while será encerrrado com break:
        if (std::abs(delta_pos) <= 3) {
            break; 
        }

        // Define a direção, onde: sentido horário= true e sentido anti-horarrio= false. Conforme definição
        // do step driver do motor, modelo "driverTB6600": 
        bool direcao = (delta_pos > 0);

        // Converte a diferença do encoder para pulsos físicos do motor (1/16), pois a resolução do passo do motor
        // é menor que a resolução do encoder:
        //int numPulsosMotor = std::abs(delta_pos) * pulso_por_revolucao / resolucao_encoder;

        //std::cout << "Numero de pulsos= " << numPulsosMotor << std::endl;

        // Chama o método de movimentação para executar a correção da posição do motor atando na GPIO da Jetso Orin nano
        darPasso(line_to_pulso, line_to_dir, direcao, 100);
    }
    //std::cout << "Posicao finalizada!! " << std::endl;
}


