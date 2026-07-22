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
#include "control_triangulacao.h"


// Simples rotina para limpar a tela:
void limparTela() {
    // \033[2J: Limpa a tela inteira
    // \033[H: Move o cursor para a posição inicial (canto superior esquerdo)
    std::cout << "\033[2J\033[H" << std::flush;
}




// Apenas habilita o PWM que controla a potencia do laser.
// O PWM é enviado a placa que converte PWM para tensão:
void ControlTriangulacao::ativaLinhaLaser(PWM &pwm){
    // Verifica se o PWM da linha laser já está ativo :
    if (!pwm.getStatus()){
        // Tenta ativar o PWM que controla a linha laser:
        if (!pwm.enable())           {
            std::cout<<" [Atenção] NÂO foi possível ativar PWM -> Controla potencia da linha laser."<< std::endl;
            return;
        }
        //std::cout << "PWM Laser ativado." << std::endl;    
    }
}


// APenas desativa o pwm da linha laser:
void ControlTriangulacao::desativaLinhaLaser(PWM &pwm){
    if (pwm.getStatus()){
        pwm.disable(); 
        //std::cout << "Desativei o laser." << std::endl; 
    }
    else{
        std::cout<<"PWM laser já está desativado." << std::endl;
    }
}

// Incrementa o valor do PWM dado em percentual, o parâmetro "use_led_potencia" é usado para limitar o duty-cycle em 10% 
// caso esteja sendo usado o LED de potencia LT2PR, para evitar danos ao led:
void ControlTriangulacao::incrementaPWM(PWM &pwm, bool useLed_LT2PR){
    int passo= 1;

    if (useLed_LT2PR){
        long dutyCycle= pwm.getDutyCycle();
        // Limita o duty-cycle em 10% caso esteja sendo usado o LED de potencia LT2PR:
        if (dutyCycle<=(10-passo)){
            dutyCycle += passo;                    
            pwm.setDutyCycle(dutyCycle);
        }  
        else
            std::cout<< "[Led LT2PR] Duty-cycle atingiu o valor máximo de 10." <<std::endl;
    }    
    else{  
        //long dutyCycle= 10;
        //pwm.setDutyCycle(dutyCycle);
        //std::cout << " [Atenção!] Laser da Osela configurado para PWM único de 50%.\n";
        long dutyCycle= pwm.getDutyCycle();

        if (dutyCycle<= 100){
            dutyCycle += passo;
            pwm.setDutyCycle(dutyCycle);
        }               
        else
            std::cout<< "Duty-Cycle= 100%)";
    }

}


// Decrementa o valor do PWM dado em percentual:
void ControlTriangulacao::decrementaPWM(PWM& pwm){ 
    int passo= 2;
    long dutyCycle= pwm.getDutyCycle();

    if (dutyCycle>= passo){
        dutyCycle -= passo;
        pwm.setDutyCycle(dutyCycle);
    } 
    else if (dutyCycle==1){
        dutyCycle= 0;
        pwm.setDutyCycle(dutyCycle);
    }
    else
        std::cout<< "Duty-Cycle= 0%)";                               
}



// Este método apenas seta o estado do LED apenas se PWM_A E PWM_B estejam ativos:
bool ControlTriangulacao::ativaLed(){
    // Verifica se o PWM_A esta ativo:
    if (!pwm_A.getStatus()){
        // Tenta ativar o PWM_A:
        if (!pwm_A.enable())           {
            std::cout<<" [Atenção] NÂO foi possível ativa PWM_A."<< std::endl;
            led.setRunning(false);
            return false;
        }    
    }

    // Verifica se o PWM_B esta ativo:
    if (!pwm_B.getStatus()){
        // Tenta ativar o PWM_B:
        if (!pwm_B.enable()){
            std::cout<<" [Atenção] NÂO foi possível ativa PWM_B." << std::endl;
            led.setRunning(false);
            return false;
        }    
    }

    led.setRunning(true);
    return true;
}



void ControlTriangulacao::desativaLed(){
    if (pwm_A.getStatus()){
        pwm_A.disable();
       // std::cout<<" PWM blink: Desativado."<< std::endl; 
    }
    else
        std::cout<<" PWM A já está desativado." << std::endl;

    if (pwm_B.getStatus()){
        pwm_B.disable();
       // std::cout<<" PWM voltage: Desativado."<< std::endl; 
    }
    else
        std::cout<<" PWM B já está desativado." << std::endl;        
    led.setRunning(false); 
}



// Esa fuanção é executada antes de tentar instanciar uma camera de eventos, para verificar se há câmeras 
// conectadas no barramento USB, evitando erros de conexão ou falhas de hardware.
// Caso não sejam detectadas cameras, o progrma é abortado.
bool ControlTriangulacao::detectaCamerasConectadas(){
    try{
        // Tenta capturar a lista de seriais de todas as câmeras conectadas:
        Metavision::DeviceDiscovery::SerialList dispositivos= Metavision::DeviceDiscovery::list();
        if (!dispositivos.empty()){
            int numDevices= dispositivos.size();
            std::cout <<  std::endl;            
            std::cout << "Cameras de Eventos Conectadas: "<< numDevices << std::endl;
            for (int ct=0; ct<numDevices; ct++){
                auto it = std::next(dispositivos.begin(), ct); 
                // Exibe apenas os 8 primeiros caracteres do número de série, que são os mais relevantes para identificação. O restante é sufixo e pode variar entre modelos ou versões.
                std::cout << " - " << it->substr(0, 11) << " | Nº Série: " << it->substr(32, 40) << std::endl;
            }
            std::cout << std::endl; 
            return true;
        }
        else{
            std::cerr << "Nenhuma câmera de eventos detectada no barramento USB." << std::endl; 
            return false;
        }
    }
    catch (const std::exception &e) {
        std::cerr << "[ERRO] Falha ao escanear barramento USB." << e.what() << std::endl;
        return false;
    }    
}


// Função que gera trem de N pulsos, onde N é definido por numPulse:
void ControlTriangulacao::pulseTrigger(int numPulse, int pin, int64_t duracaoPulso){
    std::cout << " Gerando pulso de: " << duracaoPulso << "ms no pino: "<< pin << std::endl;
    gpios_actives.piscaLed;

    for (int ctPulse=0; ctPulse<numPulse; ctPulse++){
        gpiod_line_set_value(gpios_actives.piscaLed, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(duracaoPulso));
        gpiod_line_set_value(gpios_actives.piscaLed, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(duracaoPulso));
    }
    std::cout << " Pulso finalizado." << std::endl;
}



// Inicia a camera de eventos para salvar dados em arquivo .data a partir de trigger de HW
void ControlTriangulacao::startTriggerSaveEventToFile() {
    std::string full_path="";
    try {
        // Obter data para o nome da pasta e do arquivo:
        auto agora = std::chrono::system_clock::now();
        auto tempo_t = std::chrono::system_clock::to_time_t(agora);
        struct tm *info = std::localtime(&tempo_t);

        // gera o nome do diretório:
        std::stringstream ss_pasta;
        ss_pasta << parametros_gerais.dir_to_save_event_files.c_str() << "triangulacao_" << std::put_time(info, "%d_%m_%Y");
        std::string nome_pasta = ss_pasta.str();

        // Criar a pasta se ela não existir
        if (!std::filesystem::exists(nome_pasta)) {
            std::filesystem::create_directories(nome_pasta);
        }

        // Gera o nome do arquivo de dados .raw:
        std::string serialNumber= event_cam->getSerial();
        // Elimina os 5 zeros a esquerda do inteiro com o metodo "erase()" com  "find_first_not_of()":
        serialNumber.erase(0, serialNumber.find_first_not_of('0')); 
        std::stringstream ss_time;
        ss_time << std::put_time(info, "%H%M%S");
        std::string time= ss_time.str();
        std::string filename= "data_event_cam_sn" + serialNumber + "_" + time + ".raw";
        std::string full_path= nome_pasta + '/' + filename;
        std::cout<<"salvando em eventos de varredura em: "<< full_path<< std::endl;  

        // Inicia Gravação do arquivo de dados:
        if (event_cam->startRecording(full_path)){
            // Pré-trigger: aguarda um tempo em microsegundos definido em duracao_PosTrigger_microSeg para garantir que o arquivo foi aberto e o buffer inicializou:
            std::this_thread::sleep_for(std::chrono::microseconds(parametros_gerais.duracao_pre_trigger));
            
            // ******************* Início do pulso de trigger **************** 
            // Transição do trigger para nível alto:
            gpiod_line_set_value(gpios_actives.triggerEventCam, 1);  
        }

    } catch (const std::exception &e) {
        std::cerr << "[ERRO] Falha na thread de captura: " << e.what() << std::endl;
    }
}



// Salva eventos de dados em arquico .raw a aprtir do trigger de HW:
void ControlTriangulacao::stopTriggerSaveEventToFile() {
    try {
        // ******************** Fim do pulso de trigger ******************
        // Transição do trigger para nível baixo:
        gpiod_line_set_value(gpios_actives.triggerEventCam, 0);

        // Pós-trigger: aguarda um tempo em microsegundos definido em duracao_PosTrigger_microSeg antes de fecahr o arquivo de dados: 
        std::this_thread::sleep_for(std::chrono::microseconds(parametros_gerais.duracao_pos_trigger));

        // Finaliza a Gravação e fecha arquivo de dados:
        if (event_cam->stopRecording())
            std::cout << "Gravacao de eventos finalizada. " << std::endl;
        else
            std::cout << "[ERRO] Problema ao fechar arquivo." << std::endl;
    }
    catch (const std::exception &e) {
        std::cerr << "[ERRO] Falha na thread de captura: " << e.what() << std::endl;
    }
}


// EventCamera &cam, gpiod_line *line, const PARAMETROS_GERAIS &params

// Salva eventos de dados em arquico .raw a aprtir do trigger de HW:
void ControlTriangulacao::saveData_Mono_TriggerHW() {
    try {
        // Obter data para o nome da pasta e do arquivo:
        auto agora = std::chrono::system_clock::now();
        auto tempo_t = std::chrono::system_clock::to_time_t(agora);
        struct tm *info = std::localtime(&tempo_t);

        // gera o nome do diretório:
        std::stringstream ss_pasta;
        ss_pasta << parametros_gerais.dir_to_save_event_files.c_str() << std::put_time(info, "%d_%m_%Y");
        std::string nome_pasta = ss_pasta.str();

        // Criar a pasta se ela não existir
        if (!std::filesystem::exists(nome_pasta)) {
            std::filesystem::create_directories(nome_pasta);
        }

        // Gera o nome do arquivo de dados .raw:
        std::string serialNumber= event_cam->getSerial();
        // Elimina os 5 zeros a esquerda do inteiro com o metodo "erase()" com  "find_first_not_of()":
        serialNumber.erase(0, serialNumber.find_first_not_of('0')); 
        std::stringstream ss_time;
        ss_time << std::put_time(info, "%H%M%S");
        std::string time= ss_time.str();
        std::string filename= "data_event_cam_sn" + serialNumber + "_" + time + ".raw";
        std::string full_path= nome_pasta + '/' + filename;
        std::cout<<"salvando em: "<< full_path<< std::endl; 

        // Inicia Gravação doarquivo de dados:
        if (event_cam->startRecording(full_path)){
            std::cout << "Salvando dados ....." << std::endl;

            // Pré-trigger: aguarda um tempo em microsegundos definido em duracao_PosTrigger_microSeg para garantir que o arquivo foi aberto e o buffer inicializou:
            std::this_thread::sleep_for(std::chrono::microseconds(parametros_gerais.duracao_pre_trigger));
            

            // ******************* Início do pulso de trigger **************** 
            // Transição do trigger para nível alto:
            gpiod_line_set_value(gpios_actives.triggerEventCam, 1);  

            // Mantém o pulso em nivel alto pelo tempo especificado em "duracao_pulso_trigger_microSeg":
            std::this_thread::sleep_for(std::chrono::microseconds(parametros_gerais.duracao_pulso_trigger));

            // Transição do trigger para nível baixo:
            gpiod_line_set_value(gpios_actives.triggerEventCam, 0);
            // ******************** Fim do pulso de trigger ******************

            // Pós-trigger: aguarda um tempo em microsegundos definido em duracao_PosTrigger_microSeg antes de fecahr o arquivo de dados: 
            std::this_thread::sleep_for(std::chrono::microseconds(parametros_gerais.duracao_pos_trigger));

            // Finaliza a Gravação e fecha arquivo de dados:
            if (event_cam->stopRecording())
                std::cout << "Dados salvos no arquivo: " << "\"" << full_path << "\"" << std::endl;
            else
                std::cout << "!!!ERRO ao fechar arquivo:" << std::endl;
        }

    } catch (const std::exception &e) {
        std::cerr << "[ERRO] Falha na thread de captura: " << e.what() << std::endl;
    }
}




// Método privado que efetua a cofniguração do GPIO da Jetson Orin nano:
int ControlTriangulacao::configuraGPIO_Jetson(){
    // Chama o método "get()" do objeto "configura_Jetson" da classe "ConfigJetson" definida em control_hardware.
    // Este metodo captura as informações dos pinos e lines "ativos" da Jetson:    
    auto activePins= configura_Jetson.getPinos();
    
    // Chama o método de configuração "configura_Jetson.configura_GPIO_Jetson(&chip)" para inicialização do barramento GPIO da Jetson.
    // Ela retorna uma struct contendo ponteiros com os endereços de cada linha do GPIO da Jetson para controle pelo Kernell.
    // Através desses endereços que os pinos de IO são diretamente manipulados, por exemplo, mudanças de nivel lógico.
    gpios_actives= configura_Jetson.configura_GPIO_Jetson(&chip);

    // Por segurança, verifica se as linhas "gpios_actives" foram ativadas, configuradas ok, caso contrario o programa é abortado para evitar falhas de hardware:
    if (gpios_actives.triggerEventCam == nullptr || gpios_actives.piscaLed == nullptr || gpios_actives.triggerNormalCam == nullptr || 
        gpios_actives.controlMotor_Pulso == nullptr  || gpios_actives.controlMotor_Dir == nullptr ) {
            std::cerr << "[ERRO] Falha crítica na inicialização dos GPIOs. Abortando!!!!" << std::endl;
        if (chip)
            gpiod_chip_close(chip);
        return 0;
    }
    
    return 1;
}




// Função para leitura do arquivo .json que contpem os parametros de configuração do sistema de tringualação laser, referentes ao motor e ao encoder.
// Os paramentros lidos são armazenados na struct "params" do tipo struct "PARAMETROS_GERAIS", definida em "parametros.h".
int  ControlTriangulacao::lerParametrosTriangulacaoJson(){
    std::string path= parametros_gerais.pathFileParametersTriangulation;
    
    // Abre um stream para o arquivo .json:
    std::ifstream file(path);
    if (!file.is_open()) 
        return 0;

    // Instacni aum objeto data do tipo nlohmann::json:
    nlohmann::json data;

    // Escreve o conteudo de file no objeto data:
    file >> data;

    // Variavel auxiliar apenas parametros do motor:
    auto params_aux= data["params_motor"];
    // Faz a leitura dos parametros do motor:
    for (auto &item: params_aux) {
        //std::cout << "Parametro lido " << item["name"] << "= "<<  item["value"] << std::endl;     
        if (item["name"] == "motor_pos_ang_ini") 
            parametros_gerais.motor_pos_ang_ini= item["value"];
        
        else if (item["name"] == "motor_pos_ang_fim") 
            parametros_gerais.motor_pos_ang_fim= item["value"];

        else if (item["name"] == "motor_pulsos_por_revolucao")
            parametros_gerais.motor_pulsos_por_revolucao= item["value"];

        else if (item["name"] == "motor_num_ciclos")
            parametros_gerais.motor_num_ciclos= item["value"]; 
            
        else if (item["name"] == "motor_delay_passo")
            parametros_gerais.motor_delay_passo= item["value"]; 

        else if (item["name"] == "motor_res_encoder")
            parametros_gerais.motor_res_encoder= item["value"];
    }        

    // Variavel auxiliar apenas parametros da intefaace spi:
    params_aux= data["params_spi"];
    // Faz a leitura dos parametros de inicialização da interface SPI: 
    for (auto &item: params_aux) {
        if (item["name"] == "spi_sck")
            parametros_gerais.spi_sck= item["value"];

        else if (item["name"] == "spi_num_bits")
            parametros_gerais.spi_num_bits= item["value"];   

        else if (item["name"] == "spi_device")
            parametros_gerais.spi_device= item["value"]; 
        else{
            std::cout << "[Erro] Campo " << item["name"] << " não encontrado no arquivo: "<< path << std::endl;
            return 0; // Não encontrado
        }        
    }


    // Variavel auxiliar apenas parametros da fonte laser:
    params_aux= data["params_laser"];
    // Faz a leitura dos parametros de inicialização da interface SPI:   
    for (auto &item: params_aux) {
        if (item["name"] == "pulsado")
            parametros_gerais.laser_pulsado= item["value"];

        else if (item["name"] == "passos_por_pulso")
            parametros_gerais.laser_passos_por_pulso= item["value"]; 

        else{
            std::cout << "[Erro] Campo " << item["name"] << " não encontrado no arquivo: "<< path << std::endl;
            return 0; // Não encontrado
        }        
    }


  // Variavel auxiliar apenas parametros da fonte laser:
    params_aux= data["params_pwm"];
    // Faz a leitura dos parametros de inicialização da interface SPI:   
    for (auto &item: params_aux) {
        if (item["name"] == "duty_cycle_pwm_A")
            parametros_gerais.dutyCycle_PWM_A= item["value"];

        else if (item["name"] == "duty_cycle_pwm_B")
            parametros_gerais.dutyCycle_PWM_B= item["value"]; 

        else if (item["name"] == "periodo_pwm_A")
            parametros_gerais.periodo_PWM_A= item["value"]; 

        else if (item["name"] == "periodo_pwm_B")
            parametros_gerais.periodo_PWM_B= item["value"]; 
        else{
            std::cout << "[Erro] Campo " << item["name"] << " não encontrado no arquivo: "<< path << std::endl;
            return 0; // Não encontrado
        }        
    }    

    
    // Variavel auxiliar apenas para parametros genericos:
    params_aux= data["params_generic"];
    for (auto &item: params_aux) {
        if (item["name"] == "dir_to_save")
            parametros_gerais.dir_to_save_event_files= item["value"];

         else{
            std::cout << "[Erro] Campo " << item["name"] << " não encontrado no arquivo: "<< path << std::endl;
            return 0; // Não encontrado
        }              
    }    
    return 1;
} 
      



// Função que efetua a varredura de trinagulação com laser continuo:
void ControlTriangulacao::varreduraTriangulacaoLaserContinuo(){
    std::cout << "Efetuando varredura laser" << std::endl;

    // Instancia o motor_passo passsando como paramentro o ponteiro spi:
    Motor motor_passo(&spi);

    // Atualiza os atributos do objeto "motor_paso" para controle do motor, pois o arquivo "params_triangulacao.json" pode ter sido alterado: 
    motor_passo.setPulsoPorRevolucao(parametros_gerais.motor_pulsos_por_revolucao);
    motor_passo.setResolucaoEncoder(parametros_gerais.motor_res_encoder);

    // Posiciona o motor no angulo inicial, conforme definido no parametro "params.motor_pos_ang_ini".
    // A varredura com o laser será efetuad apenas após este posicionamento:
    motor_passo.irParaPosicaoInicial(gpios_actives.controlMotor_Pulso, gpios_actives.controlMotor_Dir, parametros_gerais.motor_pos_ang_ini);

    // Converte o "angulo" da posição final, dado em graus, para passos do encoder, valor entre 0 e 16383:
    int16_t pos_final= static_cast<int16_t>((parametros_gerais.motor_pos_ang_fim*parametros_gerais.motor_res_encoder)/360.0);

    // Uma vez posicionado o motor no agnulo onicial, o PWM do laser é ativado.
    // Este sinal pwm é enviado ao conversor PWM-Tensão, pois o que controla a potencia do laser é a tensão:
    ativaLinhaLaser(pwm_B);

    int ctVarredura= 0;
    // Loop de malha fechada para incrementar o motor, passo-a-passo, até a posição final definida em :
    while(true){
        ctVarredura++;
        motor_passo.lerEncoder();
        uint16_t pos_atual_encoder= motor_passo.getPositionEncoder();
        
        // Calcula a distancai entre as posições atual e a posição desejada:
        int32_t delta_pos= pos_final - pos_atual_encoder;

        // Determina o caminho mais curto para reposicionar o motor:
        if (delta_pos> (parametros_gerais.motor_res_encoder/2)) {
            delta_pos -= parametros_gerais.motor_res_encoder;
        } 
        else if (delta_pos< -(parametros_gerais.motor_res_encoder/2)) {
            delta_pos += parametros_gerais.motor_res_encoder;
        }
        
        // Condição de parada!!!
        // Se já estiver na posição atual, considerando uma margem de histerese de ruído do encoder,
        // o loop while será encerrrado com break:
        if (std::abs(delta_pos) <= 3) {
            break; 
        }

        // Define a direção, onde: sentido horário= true e sentido anti-horarrio= false. Conforme definição
        // do step driver do motor, modelo "driverTB6600": 
        bool direcao= (delta_pos > 0);
        
        // Chama o método de movimentação para executar a correção da posição do motor atando na GPIO da Jetso Orin nano
        int delay= parametros_gerais.motor_delay_passo;
        motor_passo.darPasso(gpios_actives.controlMotor_Pulso, gpios_actives.controlMotor_Dir, direcao, delay);        
    }

    // Desativa o pwm da linha laser:
    desativaLinhaLaser(pwm_B);
    std::cout << "Varredura finalizada. Total de passo= "<< ctVarredura << std::endl;
    std::cout << std::endl;
}




// Função que efetua a varredura de trinagulação com laser pulsado:
void ControlTriangulacao::varreduraTriangulacaoLaserPulsado(){
    //std::cout << "Efetuando varredura laser" << std::endl;

    // Instancia o motor_passo passsando como paramentro o ponteiro spi:
    Motor motor_passo(&spi);

    // Atualiza os atributos do objeto "motor_paso" para controle do motor, pois o arquivo "params_triangulacao.json" pode ter sido alterado: 
    motor_passo.setPulsoPorRevolucao(parametros_gerais.motor_pulsos_por_revolucao);
    motor_passo.setResolucaoEncoder(parametros_gerais.motor_res_encoder);

    // Posiciona o motor no angulo inicial, conforme definido no parametro "params.motor_pos_ang_ini".
    // A varredura com o laser será efetuad apenas após este posicionamento:
    motor_passo.irParaPosicaoInicial(gpios_actives.controlMotor_Pulso, gpios_actives.controlMotor_Dir, parametros_gerais.motor_pos_ang_ini);

    // Converte o "angulo" da posição final, dado em graus, para passos do encoder, valor entre 0 e 16383:
    int16_t pos_final= static_cast<int16_t>((parametros_gerais.motor_pos_ang_fim*parametros_gerais.motor_res_encoder)/360.0);

    int ctVarredura= 0;
    // Loop de malha fechada para incrementar o motor, passo-a-passo, até a posição final definida em :

    // Chama a gravação do dados da câmera de eventos, que é feita em paralelo, ou seja, enquanto a câmera de eventos está gravando os 
    // dados no arquivo .raw, o programa continua executando as próximas linhas de código, sem esperar a finalização da gravação.   
    startTriggerSaveEventToFile();

    // Contador auxiliar para determinar quando o laser deve pulsar, considernado o nuero de passos por pulso definido em parametro 
    // "laser_passos_por_pulso" em "parametros.h" e no arquivo .jason: 
    int ctPassoPorPulsoLaser= 0;
    //ativaLinhaLaser(pwm);
    // Loop de varredura:


    while(true){
        //Incrementa o contador de varredura:
        ctVarredura++;
        ctPassoPorPulsoLaser++;

        // Descobre a posição do motor de passo lendo o encoder:
        motor_passo.lerEncoder();
        uint16_t pos_atual_encoder= motor_passo.getPositionEncoder();
               
        // Calcula a distancai entre as posições atual e a posição desejada:
        int32_t delta_pos= pos_final - pos_atual_encoder;

        // Determina o caminho mais curto para reposicionar o motor:
        if (delta_pos> (parametros_gerais.motor_res_encoder/2)) {
            delta_pos -= parametros_gerais.motor_res_encoder;
        } 
        else if (delta_pos< -(parametros_gerais.motor_res_encoder/2)) {
            delta_pos += parametros_gerais.motor_res_encoder;
        }
     
        // Condição de parada!!!
        // Se já estiver na posição atual, considerando uma margem de histerese de ruído do encoder, o loop while será encerrrado com break:
        if (std::abs(delta_pos) <= 3) {
            break; 
        }        

        // Define a direção, onde: sentido horário= true e sentido anti-horarrio= false. Conforme definição
        // do step driver do motor, modelo "driverTB6600": 
        bool direcao= (delta_pos > 0);
        
        // Chama o método de movimentação para executar a correção da posição do motor atando na GPIO da Jetso Orin nano
        int delay= parametros_gerais.motor_delay_passo; 
        
        
        // Da um pulso na fonte laser.
        // Este parametro pwm é enviado ao conversor PWM-Tensão, pois o que controla a potencia do laser é a tensão:
        if (ctPassoPorPulsoLaser> (parametros_gerais.laser_passos_por_pulso-1)){
            ativaLinhaLaser(pwm_B);
            //std::cout << "Ct passo por pulso laser= " << ctPassoPorPulsoLaser << std::endl;
            
            // Apenas da um wait na thread para dar tempo de setar o HW e consequentente ativar e desativar o laser:
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            // Desativa a projeção da linha laser:
            desativaLinhaLaser(pwm_B);             

            // Zera o contador de passo por pulso laser:
            ctPassoPorPulsoLaser= 0;
        }

        //Executa um passo do motor
        motor_passo.darPasso(gpios_actives.controlMotor_Pulso, gpios_actives.controlMotor_Dir, direcao, delay); 
    }

    // Chama função que para a gravação de eventos em arquivo .dat.
    stopTriggerSaveEventToFile();
    //desativaLinhaLaser(pwm);
    
    std::cout << "Total de passos no motor= "<< ctVarredura << std::endl;
    std::cout << std::endl;
}




// Esta função é responsável por configurar a visualização dos frames das câmeras de eventos e do menu no dashboard.
// Ela inicializa as ROIs para cada câmera e para o menu, configura os geradores de frames para cada câmera de eventos, e define os callbacks para atualizar os frames em tempo real.
void ControlTriangulacao::configuraFrameGenerator() {
    // Pega as dimensões da camera:
    int cam_w = event_cam->getWidth();
    int cam_h = event_cam->getHeight();

    // Define a largura do canvas que exibira o menu a direita e os frames de dados a esquerda:
    int largura_total_canvas = cam_w + configFrameGen.largura_canvas;

    // Inicializa a janela, canvas, que conterá as subjanelas, ou subcanvas, separadas para menu e dados da câmera de eventos:
    configFrameGen.canvas= cv::Mat::zeros(cam_h, largura_total_canvas, CV_8UC3);
    // Define um ROI do Canvas para cada câmera, onde os frames gerados serão exibidos, e um ROI para o menu:
    configFrameGen.canvas_events= configFrameGen.canvas(cv::Rect(0, 0, cam_w, cam_h));
    // Deine um ROI para o menu, que é um espaço reservado no canvas onde o menu será desenhado, e atualizado a cada frame:
    configFrameGen.canvas_menu= configFrameGen.canvas(cv::Rect(cam_w, 0, configFrameGen.largura_canvas, cam_h));

    // Chama o método initFrameGenerator(), do objeto "event_cam" instanciado da classe "EventCamera", para inicializar o frame gerador da camera de eventos:
    event_cam->initFrameGenerator();

    // Recupera o ponteiro do gerador nativo da câmera através do método "getGenerator()":
    generator= event_cam->getGenerator();
    
    // Configura o Gerador de frames e Callbacks
    // configFramesGen.frameGenerators é um vetor de ponteiros inteligentes para objetos do tipo "Metavision::CDFrameGenerator", 
    // que são usados para gerar os frames a partir dos eventos capturados pelas câmeras de eventos.
    // configFG.frameGenerators.push_back(std::make_unique<Metavision::CDFrameGenerator>(cam_w, cam_h));

    if (generator){
        // Define a duração temporal, janela de temporal, que formara um frame de eventos.
        // A duração deste frame é dada em microsegundos e está definida em "configFramesGen.janelaTemporalFrame", que é um parâmetro configurável pelo usuário.    
        generator->set_display_accumulation_time_us(configFrameGen.janelaTemporalFrame);

        // Define os parametros de cores:
        cv::Scalar backGroud_Color= cv::Scalar(0, 0, 0); 
        cv::Scalar eventsON_Color= cv::Scalar(255, 0, 0);
        cv::Scalar eventsOFF_Color= cv::Scalar(0, 0, 255);

        // Configura a cor do eventos e do back Groud, o ultimo parametro true força a saida em 3 canais:
        generator->set_colors(backGroud_Color, eventsON_Color, eventsOFF_Color, true);

        // Start a Thread para o configFramesGen.frameGenerators gerenciada pelo próprio SDK da Metavision, que processa os eventos e gera os frames de eventos 
        // a cada janela temporal definida, ela chama a callback para atualizar o frame exibido no dashboard. 
        // A callback é a própria função lambda definida dentro do start(), onde o frame gerado é copiado para o ROI do canvas reservado para os dados 
        // da câmera de eventos, ou seja, "ctx.canvas_events".
        // A callback é chamada com periodo (frequencia) definido em "ctx.freqCallback_frameGenerator", que é um parâmetro configurável pelo usuário.
        generator->start(configFrameGen.freqCallback_frameGenerator, [this](const Metavision::timestamp &ts, const cv::Mat &frame) {
            if (configFrameGen.showViewerEvents) {
                // Trava a leitura de dados via mutex para evitar condições de corrida, garantindo que o frame seja copiado de fomra segura para o canvas.
                std::unique_lock<std::mutex> lock(configFrameGen.mutex);
                // Copia o frame de eventos para exibição pelo canvas reservado para os dados da câmera de eventos:
                frame.copyTo(configFrameGen.frame);
            }
        });
    }

    // Esta Callback recebe Eventos Brutos via USB. Ela é chamada de forma assíncrona pelo driver da SDK Metavision sempre que
    // um novo pacote de dados chega via USB a partir da câmera de eventos.
    // "ev_begin": Ponteiro para o primeiro evento válido do pacote na RAM.
    // "ev_end": Ponteiro que aponta para o fim do pacote na RAM.
    // De forma geral, neste trecho de código está sendo definida a callback que será chamada pela thread do SDK Metavisio, 
    // que é iniciada quando for executado o método "event_cam.start()". 
    event_cam->setCDCallback([this](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
        if (configFrameGen.showViewerEvents && generator) {
            // Insere os dados brutos no frameGenerator que irá acumular os eventos recebidos durante a janela temporal definida, 
            // e a cada janela temporal ele irá gerar um frame de eventos atualizado, chamando a callback definida em "ctx.frameGenerators[0]...".
            generator->add_events(ev_begin, ev_end);
        }
    });


    // Define o container de string referenta ao contúdo do canvas_menu:
    std::vector<std::string> itens = {"--- Menu de Usuario ---", 
                                      "1 - Ler Biases da evecam", 
                                      "2 - Gravar Biase na evecam", 
                                      "3 - Trigger REC", 
                                      "4 - Blink LED", 
                                      "5 - Cam. Convencional", 
                                      "6 - Verredura Triangulacao",
                                      "-----------------------", 
                                      "+ / - : Duty Cycle Tensao",
                                      "> / < : Duty Cycle Blink Light",
                                      " ",
                                      "Esc - Sair"};
    
    // Configura a cor de fundo do canvas do menu:
    configFrameGen.canvas_menu.setTo(cv::Scalar(220, 220, 220));

    // // Desenha o Menu que será exibido no "canvas_menu", lado direito do canvas geral::
    for (size_t i = 0; i < itens.size(); ++i) {
        cv::putText(configFrameGen.canvas_menu, 
                    itens[i], 
                    cv::Point(20, 40 + i * 35), 
                    cv::FONT_HERSHEY_SIMPLEX, 
                    0.7, 
                    cv::Scalar(0, 0, 0), 
                    1, 
                    cv::LINE_AA);
    }

    std::string window_name= configFrameGen.window_name;
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
}



int ControlTriangulacao::configuraCameraConvencional(){

    // Instanciar o Singleton: é uma instância única do sistema que gerencia a comunicação com o hardware para acessar a camera.
    // Carrega a Camada de Transporte: Inicializa os drivers e protocolos (USB3, GigE) necessários para detectar e conversar com as câmeras.
    // Ponto de Entrada: É o objeto obrigatório para listar dispositivos e gerenciar o ciclo de vida da SDK.
    Spinnaker::SystemPtr system= Spinnaker::System::GetInstance();

    // Varredura das portas: Escaneia as interfaces (USB/GigE) em busca de câmeras conectadas.
    // Cria uma lista contendo referências (objetos CameraPtr) para todos os dispositivos encontrados.
    // Permite localizar uma câmera específica, por exemplo pelo nº de Serie para começar a operá-la.
    Spinnaker::CameraList camList= system->GetCameras();

    // Instancia o  objeto da camera convencional apenas se estiver habilitado este procedimento.
    if (parametros_gerais.useCamera_Conv){

        // Busca Seletiva: Percorre a lista de câmeras detectadas procurando o identificador único, Serial Number, definido nos parametros_gerais.
        // Verifica se a câmera desejada está conectada e disponível antes de iniciar a operação.
        Spinnaker::CameraPtr pCamBase = camList.GetBySerial(parametros_gerais.serialNumber_conv_cam_01);

        // Testa se a camera com o referido nº de séri foi encontrada:
        if (!pCamBase.IsValid()) {
            std::cerr << "Câmera não detectada!" << std::endl;
            return 0;
        }

        // Conversão de Tipo: Transforma o ponteiro genérico da SDK, Spinnaker::Camera*, no tipo definido pela minha classe ConvCamera: 
        // Permite acesso aos métodos e atributos personalizados especificos da classe ConvCamera e também da classe original da FLIR, que foi herdada.
        // Desta forma, a classe ConmvCamera terá acesso a toda as funções e metodos da classe Spinnaker::Camera:
        conv_cam_01 = static_cast<ConvCamera*>(pCamBase.get());
        
        // Se ok, será exibida a configuraçã da camera convencional 
        if (conv_cam_01){
            conv_cam_01->Init();
            std::cout<< std::endl;
            std::cout<< "*** Câmera convencional: ***"<< std::endl;
            conv_cam_01->exibir_modelo_camera();
            std::cout<< std::endl;
            conv_cam_01->DeInit();
        }
    }    
    return 1;
}



int ControlTriangulacao::configuraCameraDeEventos(){
    // Chama metodo que detecta as cameras de eventos conectadas ao device USB.
    // Se não detectar camera de eventos no barramente USB, o programa é abortado para evitar falhas de hardware:
    if (!detectaCamerasConectadas()){
        return 0;
    }

    // Aloca a câmera de eventos na memória apenas se ela foi detectada no barramento. 
    event_cam= std::make_unique<EventCamera>(parametros_gerais.serialNumber_event_cam3, "Event Mono", true);

    //EventCamera event_cam= EventCamera(parametros_gerais.serialNumber_event_cam3, "Event Mono", true);


    // Configura alguns parametros iniciais para cada câmera de eventos, tais como: biases, trigger e sincronismo de hardware, 
    // usando as funções da classe EventCamera, que por sua vez utilizam a API do SDK Metavision.
    // COnfigura, carrega, os biases na camera de eventos definidos em setings.json:
    if (!event_cam->setBias(parametros_gerais)){
        std::cerr << "Câmera de eventos não detectada!" << std::endl;
        return -1;
    }

    // IMPORTANTE!!! Esta função habilita o Trigger via HAL da API:
    event_cam->enableHardwareTrigger();

    return 1;
}




// Mépetodo responsável por iniclizar todo o hardware:
// - GPIO da Jetson,
// - PWM da Jetosn
// - SPI da Jetson, 
// - Camera de eventos
// - camera convencional
// - Frame generator para exibição dos eventos, imagens e menu de usuario. 
int ControlTriangulacao::inicializaHardware(){
    // Carregar os parametros para controle do motor lendo o arquivo "params_triagulacao.json":
    // Embora esses parametros sejam gerados em tempo de compilação diretamente na struct "parametros_gerais" declarada em "parametros.h",
    // a leitura do .json possibilita alterar o valor desses paramentros em tempo de execução, basta editar o .json e chamar a função para alterar essses valores.
    if (!lerParametrosTriangulacaoJson()){
        std::cout << "[Erro] Não foi possível abrir arquivo .json" << std::endl;
        return -1;
    }

    //************************** Configuração do GPIO da Jetson Orin Nano *************************/ 
    // Chama metodo privado para configurar o GPIO da Jetson:
    if (!configuraGPIO_Jetson()){
        std::cout << "[Erro] Não foi possível configuar o GPIO da Jetson!"<< std::endl;
        return -1;
    }



    //********** Configuração da camera convencional FLIR (Model BlackFly BFS-U3-04S2M) **********/
    if (!configuraCameraConvencional()){
        std::cout << "[Erro] Camera convencional não pode ser inicializada." << std::endl; 
    }


    //***************************** Configuração da interface SPI ********************************/   
    // Metodo que abre e iniciliza o objeto "spi" da "class SPIDevice", definido em "control_hardware.h", 
    // que controla a interface SPI, os paramentros estão definidos em "parametros.h": 
    if (!spi.inicializa(parametros_gerais.spi_device, parametros_gerais.spi_mode, parametros_gerais.spi_sck, parametros_gerais.spi_num_bits)){
        std::cout << "[Erro] não foi possivel inicair a interface SPI." << std::endl;
        return -1;
    }

    // std::cin.get();


    //********************* Configuração Camera de eventos SilkyEvCam Cebtury Arks **************************/   
    if (!configuraCameraDeEventos()){
        std::cout << "[Erro] Camera de eventos não pode ser inicializada." << std::endl;
        return -1;
    }


    //**************** Configuração o Frame Genarator para visualização do eventos em real time *************/ 
    // Função que configura o frameGenerator, responsável por montar os frames de eventos que serão exibdos no canvas, janela de visualização via OpenCV:
    configuraFrameGenerator();


    /************************ Dispara o funcionamenoe das câmeras de eventos ********************************/
    // A partir deste ponto, as câmeras de eventos começam a capturar e gerar frames, 
    // que são processados pelos geradores de frames e exibidos no dashboard em tempo real.
    // A contagem do laço for é invertida porque a câmera SLAVE deve ser iniciada antes da MASTER para 
    // garantir que ela esteja pronta para receber os sinais de sincronismo e trigger da master.
    event_cam->callStart();

    return 1;
}



// Metodo que finaliza o aplicativo fechando o HW com segurnaça:
void ControlTriangulacao::finalizaHardware(){
    // Fecha a câmera:  
    event_cam->stop();

    // Fecha a SPI:
    spi.finaliza();

    // Para o gerador de frame:
    if (event_cam->getGenerator()) {
        event_cam->getGenerator()->stop();
    }

    // Fecha todas as janelas:    
    cv::destroyAllWindows();

    // Desabilita o pwm
    pwm_A.disable();
    pwm_B.disable();

}

// Loop principal while(), onde o controle de opções é efetuado pela máquina Switch-Case:
void ControlTriangulacao::executaLoopPrincipal(){
    bool running = true;

    // Loop principal:
    while(running) {
        // Aguarda o usuario efetuar a escolham, digitar um tecla, por 1 ms:
        int key = cv::waitKey(1);
        //std::cout << "Código numérico da tecla: " << key << std::endl;

        // Se o visulaizador estiver habilitado, exibe o frame de eventos no dashboard:
        if (configFrameGen.showViewerEvents) {
            // Insere um Mtex para evitar conflitos de acesso ao buffer de dados de eventos:
            std::unique_lock<std::mutex> lock(configFrameGen.mutex);

            // Testa se o frame de dados está pronto para se exibido, ou seja, se o frame gerado pelo frameGenerator da câmera de eventos foi
            // copiado para o canvas reservado para os dados da câmera de eventos, respeitando a duração definida em "freqCallback_frameGenerator"
            // Conforme configurado na função configuraFrameGenerator(), especificamente em:
            // generator->start(configFG.freqCallback_frameGenerator, [&configFG](const Metavision::timestamp &ts, const cv::Mat &frame) {...}:
            if (!configFrameGen.frame.empty()) {
                // CCopia os dados de eventos convertendo explicitamente para o local de exibição de eventos, canvas_events, um ROI canvas:
                configFrameGen.frame.copyTo(configFrameGen.canvas_events);

                // Apenas insere um texto no frame, canvas, de eventos da câmera:
                cv::putText(configFrameGen.canvas_events, "EventCam", cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
            }
        }

        // Exibe o Dashboard consolidado
        cv::imshow(configFrameGen.window_name, configFrameGen.canvas);

        // Máquina de estados do menu principal:
        if (key!=-1){
            char my_char= static_cast<char>(key);

            switch (my_char)
            {
                case '1':
                case 177:
                {
                        // Efetua a leitura dos biases da camera de eventos:
                        event_cam->readCameraBiases();
                        break;
                }

                case '2':
                case 178:
                        // Chama função para configurar, enviar, os biases à camera de eventos:
                        event_cam->readCameraBiases();
                        break;

                case '3':
                case 179:
                        {
                            // Le o arquivo "params_triagulacao.json" para atualizar os paramentros usados para tratar o controle do motor.
                            // Isto possibilita alterar os paramentros em tempo de execução.
                            if (!lerParametrosTriangulacaoJson()){
                                std::cout << "[Erro] Não foi possível abrir arquivo .json" << std::endl;
                                return;
                            }

                            // Se estiver diferente o valor do dutycycle sera atulzizado para o valor definido no .json:
                            if (pwm_B.getDutyCycle()!=parametros_gerais.dutyCycle_PWM_B){
                                //std::cout << "Valor atual do duty-cycle do PWM do laser= " << pwm_PowerLight.getDutyCycle() <<std::endl;
                                pwm_B.setDutyCycle(parametros_gerais.dutyCycle_PWM_B);
                                //std::cout << "Novo valor do duty cycle do PWM do laser = " << pwm_PowerLight.getDutyCycle() <<std::endl;
                            }

                            // Usa-se uma thread com função lambda, onde:
                            // [&] = Captura: ela captura todas as variáveis e métodos, por referencia, no escopo de main(), pois ela utilzia várias funções e variáveis;
                            // () = Parâmetros: Sem nenhum parâmetro como argumento;
                            // {} = Corpo: onde são chamadas as funções.:
                            std::thread t([&]() {
                                for (int ctCiclo=0;  ctCiclo < parametros_gerais.numero_ciclos_trigger; ctCiclo++){
                                    // Primeira ativa a projeção da luz estruturada:
                                    ativaLed();
                                    // Chama a gravação do dados da câmera de eventos, que é feita em paralelo, ou seja, enquanto a câmera de eventos está gravando os
                                    // dados no arquivo .raw, o programa continua executando as próximas linhas de código, sem esperar a finalização da gravação.
                                    saveData_Mono_TriggerHW();
                                    // Por ultimo, desativa o projeção de luz estruturada:
                                    desativaLed();
                                }
                            });

                            // Desacoplar a thread para que o viewer não trave
                            t.detach();
                            break;
                        }

                case '4':
                case 180:
                        {
                            // Le o arquivo "params_triagulacao.json" para atualizar os paramentros usados para tratar o controle do motor.
                            // Isto possibilita alterar os paramentros em tempo de execução.
                            if (!lerParametrosTriangulacaoJson()){
                                std::cout << "[Erro] Não foi possível abrir arquivo .json" << std::endl;
                                return;
                            }

                            // Se estiver diferente o valor do dutycycle sera atulzizado para o valor definido no .json:
                            if (pwm_B.getDutyCycle()!=parametros_gerais.dutyCycle_PWM_B){
                                //std::cout << "Valor atual do duty-cycle do PWM do laser= " << pwm_PowerLight.getDutyCycle() <<std::endl;
                                pwm_B.setDutyCycle(parametros_gerais.dutyCycle_PWM_B);
                                //std::cout << "Novo valor do duty cycle do PWM do laser = " << pwm_PowerLight.getDutyCycle() <<std::endl;
                            }

                            if (!led.getRunning())
                                ativaLed();
                            else
                                desativaLed();
                            break;
                        }

                case '5':
                case 181:
                        {
                            // Captura uma imagem pela câmera convencional:
                            if (conv_cam_01){
                                conv_cam_01->Init();
                                conv_cam_01->capturarImagem();
                                conv_cam_01->DeInit();
                            }
                            else
                                std::cout << "Câmera convencional não instanciada." << std::endl;
                            break;
                        }

                case '6':
                case 182:
                        {
                            limparTela();

                            // Le o arquivo "params_triagulacao.json" para atualizar os paramentros usados para tratar o controle do motor.
                            // Isto possibilita alterar os paramentros em tempo de execução.
                            if (!lerParametrosTriangulacaoJson()){
                                std::cout << "[Erro] Não foi possível abrir arquivo .json" << std::endl;
                                return;
                            }

                            // Se estiver diferente o valor do dutycycle sera atulzizado para o valor definido no .json:
                            if (pwm_B.getDutyCycle()!=parametros_gerais.dutyCycle_PWM_B){
                                //std::cout << "Valor atual do duty-cycle do PWM do laser= " << pwm_PowerLight.getDutyCycle() <<std::endl;
                                pwm_B.setDutyCycle(parametros_gerais.dutyCycle_PWM_B);
                                //std::cout << "Novo valor do duty cycle do PWM do laser = " << pwm_PowerLight.getDutyCycle() <<std::endl;
                            }

                            // Incia thread para varredura da triangulação laser:
                            std::thread thread_motor([&]() {
                                // testa se usa o laser pulsado ou continuo, parametros lido do arquivo .jason e armazenado em parametros_gerais:
                                if (parametros_gerais.laser_pulsado)
                                   varreduraTriangulacaoLaserPulsado();
                                else
                                   varreduraTriangulacaoLaserContinuo();
                            });
                            thread_motor.detach();
                            break;
                        }

                case '.':
                case '>': // Incrementa o pwm que controla o tempo de atuação do Led/laser, duração do blink:
                        incrementaPWM(pwm_A, parametros_gerais.useLed_LT2PR);
                        break;
                        /*
                        {
                           if (parametros_gerais.useLed_LT2PR)
                                incrementaPWM(pwm_A, parametros_gerais.useLed_LT2PR);
                           else
                                std::cout << " [Atenção!] Use (+) para incrementar o Duty Cicle do laser.\n";
                          break;
                        }
                          */
                case ',':
                case '<': // Decrementa o pwm que controla o tempo de atuação do Led/laser, duração do blink:
                        decrementaPWM(pwm_A);
                        break;
                        /*
                        {
                            if (parametros_gerais.useLed_LT2PR)
                                decrementaPWM(pwm_A);
                            else
                                std::cout << " [Atenção!] Use (-) para decrementar o Duty Cicle do laser.\n";
                            break;
                        }
                        */

                case 171: // tecla + do bloco numerico
                case '+':
                case '=': // Incrementa o pwm que controla a tensão analogica do Led/laser (0 a 10V) ou laser (o a 5V):
                        incrementaPWM(pwm_B, false);
                        break;

                case 173: // codigo - do bloco numerico
                case '-':
                case '_': // Decremento o pwm que controla a tensão analogica do Led/laser (0 a 10V) ou laser (o a 5V):
                        decrementaPWM(pwm_B);
                        break;

                case 'l':
                case 'L':
                        // Chama função para limpar o terminal:
                        limparTela();
                        parametros_gerais.hab_exibe_menu= true;
                        break;

                case 'q':
                case 'Q':
                case 27: // ESC
                        running = false;
                        std::cout <<std::endl;
                        break;

                default:
                        std::cout << "Comando invalido!" << std::endl;
                        break;
            }
        }
    }
}
