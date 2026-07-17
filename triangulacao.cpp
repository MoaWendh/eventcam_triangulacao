// Autor: Moacir Wendhausen    
// Projeto: VORIS
// Data: 10/01/2026
// Controle aquisição: sistema estéreo baseado em duas câmeras de eventos SylkEvCam da Cenury Arks.
// Programa principal que controla a captura de dados das câmeras de eventos e convencionais, bem como o controle do trigger por hardware e do LED de iluminação.
// Apresenta um menu interativo no terminal para controle das opções baseado nas libs do OpenCV, e um dashboard visual, também OpenCV, para exibir os frames 
// capturados e o menu de controle em tempo real.
// São utilizados dosi canais de PWM da Jetson Orina Nano para controlar o blink do LED e a potência do LED.
// O trigger por hardware é gerado usando os GPIOs da Jetson Orin Nano, controlados via a interface Sysfs do Linux.
// Os principais parâmetros estão definidos no header "parametros.h", como os tempos de trigger, números de série das câmeras, configurações de PWM, entre outros.

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
#include "controlIO.h"
#include "parametros.h"


// Simples rotina para limpar a tela:
void limparTela() {
    // \033[2J: Limpa a tela inteira
    // \033[H: Move o cursor para a posição inicial (canto superior esquerdo)
    std::cout << "\033[2J\033[H" << std::flush;
}


// Função que gera trem de N pulsos, onde N é definido por numPulse:
void pulseTrigger(int numPulse, gpiod_line *line, int pin, int64_t duracaoPulso){
    std::cout << " Gerando pulso de: " << duracaoPulso << "ms no pino: "<< pin << std::endl;

    for (int ctPulse=0; ctPulse<numPulse; ctPulse++){
        gpiod_line_set_value(line, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(duracaoPulso));
        gpiod_line_set_value(line, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(duracaoPulso));
    }
    std::cout << " Pulso finalizado." << std::endl;
}



// Salva eventos de dados em arquico .raw a aprtir do trigger de HW:
void saveData_Mono_TriggerHW(EventCamera &cam, gpiod_line *line, const PARAMETROS_GERAIS &params) {
    try {
        // Obter data para o nome da pasta e do arquivo:
        auto agora = std::chrono::system_clock::now();
        auto tempo_t = std::chrono::system_clock::to_time_t(agora);
        struct tm *info = std::localtime(&tempo_t);

        // gera o nome do diretório:
        std::stringstream ss_pasta;
        ss_pasta << "data_evecam_" << std::put_time(info, "%d_%m_%Y");
        std::string nome_pasta = ss_pasta.str();

        // Criar a pasta se ela não existir
        if (!std::filesystem::exists(nome_pasta)) {
            std::filesystem::create_directory(nome_pasta);
        }

        // Gera o nome do arquivo de dados .raw:
        std::string serialNumber= cam.getSerial();
        std::stringstream ss_time;
        ss_time << std::put_time(info, "%H%M%S");
        std::string time= ss_time.str();
        std::string filename= "evecam_sn" + serialNumber + "_" + time + ".raw";
        std::string full_path= nome_pasta + '/' + filename;
        std::cout<<"salvando em: "<< full_path<< std::endl; 

        // Inicia Gravação doarquivo de dados:
        if (cam.startRecording(full_path)){
            std::cout << "Salvando dados ....." << std::endl;

            // Pré-trigger: aguarda um tempo em microsegundos definido em duracao_PosTrigger_microSeg para garantir que o arquivo foi aberto e o buffer inicializou:
            std::this_thread::sleep_for(std::chrono::microseconds(params.duracao_pre_trigger));
            

            // ******************* Início do pulso de trigger **************** 
            // Transição do trigger para nível alto:
            gpiod_line_set_value(line, 1);  

            // Mantém o pulso em nivel alto pelo tempo especificado em "duracao_pulso_trigger_microSeg":
            std::this_thread::sleep_for(std::chrono::microseconds(params.duracao_pulso_trigger));

            // Transição do trigger para nível alto:
            gpiod_line_set_value(line, 0);
            // ******************** Fim do pulso de trigger ******************

            // Pós-trigger: aguarda um tempo em microsegundos definido em duracao_PosTrigger_microSeg antes de fecahr o arquivo de dados: 
            std::this_thread::sleep_for(std::chrono::microseconds(params.duracao_pos_trigger));

            // Finaliza a Gravação e fecha arquivo de dados:
            if (cam.stopRecording())
                std::cout << "Dados salvos no arquivo: " << "\"" << full_path << "\"" << std::endl;
            else
                std::cout << "!!!ERRO ao fechar arquivo:" << std::endl;
        }

    } catch (const std::exception &e) {
        std::cerr << "[ERRO] Falha na thread de captura: " << e.what() << std::endl;
    }
}



// Este método apenas seta o estado do LED apenas se PWM_A E PWM_B estejam ativos:
bool ativaLedLight(LightController &led, PWM &pwm_A, PWM &pwm_B){
    // Verifica se o PWM_A esta ativo:
    if (!pwm_A.getStatus()){
        // Tenta ativar o PWM_A:
        if (!pwm_A.enable())           {
            std::cout<<" [Atenção] NÂO foi possível ativa PWM_A -> Controla blink led."<< std::endl;
            led.setRunning(false);
            return false;
        }    
    }

    // Verifica se o PWM_B esta ativo:
    if (!pwm_B.getStatus()){
        // Tenta ativar o PWM_B:
        if (!pwm_B.enable()){
            std::cout<<" [Atenção] NÂO foi possível ativa PWM_B -> Controla da tensão do led" << std::endl;
            led.setRunning(false);
            return false;
        }    
    }

    led.setRunning(true);
    return true;
}



void desativaLedLight(LightController &led, PWM& pwm_A, PWM& pwm_B){
    if (pwm_A.getStatus()){
        pwm_A.disable();
       // std::cout<<" PWM blink: Desativado."<< std::endl; 
    }
    else
        std::cout<<" PWM blink já está desativado." << std::endl;

    if (pwm_B.getStatus()){
        pwm_B.disable();
       // std::cout<<" PWM voltage: Desativado."<< std::endl; 
    }
    else
        std::cout<<" PWM voltage já está desativado." << std::endl;        
    led.setRunning(false); 
}



// Incrementa o valor do PWM dado em percentual, o parâmetro "use_led_potencia" é usado para limitar o duty-cycle em 10% 
// caso esteja sendo usado o LED de potencia LT2PR, para evitar danos ao led:
void incrementaPWM(PWM& pwm, const std::string funcao_PWM, bool use_led_potencia){
    int passo= 1;

    if (use_led_potencia){
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
void decrementaPWM(PWM& pwm, const std::string funcao_PWM){ 
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


// Verificação se os pinos forma configurados ok, por segurança:
int confirma_gpios_actives(GPIO_Lines &gpios_actives, gpiod_chip *chip){
    if (gpios_actives.triggerEventCam == nullptr || gpios_actives.piscaLed == nullptr || gpios_actives.triggerNormalCam == nullptr || 
        gpios_actives.controlMotor_Pulso == nullptr  || gpios_actives.controlMotor_Dir == nullptr ) {
            std::cerr << "[ERRO] Falha crítica na inicialização dos GPIOs. Abortando!!!!" << std::endl;
        if (chip)
            gpiod_chip_close(chip);
        return 0;
    }
    return 1;
}



// Esa fuanção é executada antes de tentar instanciar uma camera de eventos, para verificar se há câmeras 
// conectadas no barramento USB, evitando erros de conexão ou falhas de hardware.
// Caso não sejam detectadas cameras, o progrma é abortado.
bool detectaCamerasConectadas(){
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



// Este menu é exibido no terminal do OpenCV junamente com os frames capturados pela câmera convencional, 
// para facilitar a visualização e controle das opções de trigger e configuração do LED. 
// Ele é desenhado diretamente sobre o frame da câmera convencional, utilizando as funções de desenho do OpenCV, 
// como putText e circle. O menu inclui as opções de controle, bem como um indicador "LIVE" que pisca para mostrar que a captura está ativa. 
// A função MenuFrame é chamada a cada frame capturado pela câmera convencional para atualizar o menu em tempo real.
void MenuFrame(const cv::Mat& input, cv::Mat& output) {
    if (input.empty()) return;

    int largura_menu = 300;

    // 1. Garante que o input seja convertido para 3 canais (Colorido) 
    // para podermos desenhar em Verde/Vermelho.
    cv::Mat input_color;
    if (input.channels() == 1) {
        cv::cvtColor(input, input_color, cv::COLOR_GRAY2BGR);
    } else {
        input_color = input;
    }

    // 2. Cria a moldura com a borda preta à direita
    cv::copyMakeBorder(input_color, output, 0, 0, 0, largura_menu, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    // 3. Configurações de texto
    int x_start = input_color.cols + 20;
    int y_start = 40;
    int espacamento = 30;

    std::vector<std::string> itens = {
        "*** MENU DE CONTROLE ***",
        "1 - Ler biases",
        "2 - Gravar biases",
        "3 - Trigger (Gravacao)",
        "4 - Start/Stop Blink LED",
        "5 - Captura Convencional",
        "-----------------------",
        "+ / - : Duty Cycle Tensão",
        "> / < : Duty Cycle Blink Light",
        "L : Limpa Tela",
        "Q : Sair"
    };

    // 4. Desenha o texto
    for (size_t i = 0; i < itens.size(); ++i) {
        cv::putText(output, 
                    itens[i], 
                    cv::Point(x_start, y_start + (int)(i * espacamento)), 
                    cv::FONT_HERSHEY_SIMPLEX, 
                    0.5, 
                    cv::Scalar(0, 255, 0), // Verde
                    1, 
                    cv::LINE_AA);
    }

    // 5. Indicador "LIVE" (Círculo vermelho piscando no canto superior direito)
    // Usamos o tempo do sistema para fazer piscar
    auto milis = std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::system_clock::now().time_since_epoch()).count();
    
    if ((milis / 500) % 2 == 0) { // Pisca a cada 500ms
        cv::circle(output, cv::Point(output.cols - 20, 20), 7, cv::Scalar(0, 0, 255), -1);
        cv::putText(output, "LIVE", cv::Point(output.cols - 60, 25), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
    }
}




// Esta função é responsável por configurar a visualização dos frames das câmeras de eventos e do menu no dashboard.
// Ela inicializa as ROIs para cada câmera e para o menu, configura os geradores de frames para cada câmera de eventos, e define os callbacks para atualizar os frames em tempo real.
void configuraFrameGenerator(configFrameGenerator &configFG, EventCamera &event_cam) {
    // Pega as dimensões da camera:
    int cam_w = event_cam.getWidth();
    int cam_h = event_cam.getHeight();

    // Define a largura do canvas que exibira o menu a direita e os frames de dados a esquerda:
    int largura_total_canvas = cam_w + configFG.largura_canvas;

    // Inicializa a janela, canvas, que conterá as subjanelas, ou subcanvas, separadas para menu e dados da câmera de eventos:
    configFG.canvas= cv::Mat::zeros(cam_h, largura_total_canvas, CV_8UC3);
    // Define um ROI do Canvas para cada câmera, onde os frames gerados serão exibidos, e um ROI para o menu:
    configFG.canvas_events= configFG.canvas(cv::Rect(0, 0, cam_w, cam_h));
    // Deine um ROI para o menu, que é um espaço reservado no canvas onde o menu será desenhado, e atualizado a cada frame:
    configFG.canvas_menu= configFG.canvas(cv::Rect(cam_w, 0, configFG.largura_canvas, cam_h));

    // Chama o método initFrameGenerator(), do objeto "event_cam" instanciado da classe "EventCamera", para inicializar o frame gerador da camera de eventos:
    event_cam.initFrameGenerator();

    // Recupera o ponteiro do gerador nativo da câmera através do método "getGenerator()":
    auto* generator= event_cam.getGenerator();
    
    // Configura o Gerador de frames e Callbacks
    // configFramesGen.frameGenerators é um vetor de ponteiros inteligentes para objetos do tipo "Metavision::CDFrameGenerator", 
    // que são usados para gerar os frames a partir dos eventos capturados pelas câmeras de eventos.
    // configFG.frameGenerators.push_back(std::make_unique<Metavision::CDFrameGenerator>(cam_w, cam_h));

    if (generator){
        // Define a duração temporal, janela de temporal, que formara um frame de eventos.
        // A duração deste frame é dada em microsegundos e está definida em "configFramesGen.janelaTemporalFrame", que é um parâmetro configurável pelo usuário.    
        generator->set_display_accumulation_time_us(configFG.janelaTemporalFrame);

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
        generator->start(configFG.freqCallback_frameGenerator, [&configFG](const Metavision::timestamp &ts, const cv::Mat &frame) {
            if (configFG.showViewerEvents) {
                // Trava a leitura de dados via mutex para evitar condições de corrida, garantindo que o frame seja copiado de fomra segura para o canvas.
                std::unique_lock<std::mutex> lock(configFG.mutex);
                // Copia o frame de eventos para exibição pelo canvas reservado para os dados da câmera de eventos:
                frame.copyTo(configFG.frame);
            }
        });
    }

    // Esta Callback recebe Eventos Brutos via USB. Ela é chamada de forma assíncrona pelo driver da SDK Metavision sempre que
    // um novo pacote de dados chega via USB a partir da câmera de eventos.
    // "ev_begin": Ponteiro para o primeiro evento válido do pacote na RAM.
    // "ev_end": Ponteiro que aponta para o fim do pacote na RAM.
    // De forma geral, neste trecho de código está sendo definida a callback que será chamada pela thread do SDK Metavisio, 
    // que é iniciada quando for executado o método "event_cam.start()". 
    event_cam.setCDCallback([&configFG, generator](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
        if (configFG.showViewerEvents && generator) {
            // Insere os dados brutos no frameGenerator que irá acumular os eventos recebidos durante a janela temporal definida, 
            // e a cada janela temporal ele irá gerar um frame de eventos atualizado, chamando a callback definida em "ctx.frameGenerators[0]...".
            generator->add_events(ev_begin, ev_end);
        }
    });


    // Define o container de string referenta ao contúdo do canvas_menu:
    std::vector<std::string> itens = {"--- Menu de Usuario ---", 
                                      "1 - Ler Biases", 
                                      "2 - Gravar Biases", 
                                      "3 - Trigger REC", 
                                      "4 - Blink LED", 
                                      "5 - Cam. Convencional", 
                                      "6 - Ler encoder",
                                      "-----------------------", 
                                      "+ / - : Duty Cycle Tensao",
                                      "> / < : Duty Cycle Blink Light",
                                      " ",
                                      "Esc - Sair"};
    
    // Configura a cor de fundo do canvas do menu:
    configFG.canvas_menu.setTo(cv::Scalar(220, 220, 220));

    // // Desenha o Menu que será exibido no "canvas_menu", lado direito do canvas geral::
    for (size_t i = 0; i < itens.size(); ++i) {
        cv::putText(configFG.canvas_menu, 
                    itens[i], 
                    cv::Point(20, 40 + i * 35), 
                    cv::FONT_HERSHEY_SIMPLEX, 
                    0.7, 
                    cv::Scalar(0, 0, 0), 
                    1, 
                    cv::LINE_AA);
    }

    std::string window_name= configFG.window_name;
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
}



// Função para leitura do arquivo .json que contpem os parametros de configuração do sistema de tringualação laser, referentes ao motor e ao encoder.
// Os paramentros lidos são armazenados na struct "params" do tipo struct "PARAMETROS_GERAIS", definida em "parametros.h".
int  lerParametrosTriangulcaoJson(PARAMETROS_GERAIS &params){
    std::string path= params.pathFileParametersTriangulation;
    
    // Abre um stream para o arquivo .json:
    std::ifstream file(path);
    if (!file.is_open()) 
        return 0;

    // Instacni aum objeto data do tipo nlohmann::json:
    nlohmann::json data;

    // Escreve o conteudo de file no objeto data:
    file >> data;

    // Navega na estrutura: ll_biases_state -> bias (que é um array)
    auto params_aux= data["params"];
    // std::cout << std::endl;
    for (auto &item: params_aux) {
        //std::cout << "Parametro lido " << item["name"] << "= "<<  item["value"] << std::endl;
        
        if (item["name"] == "motor_pos_ang_ini") 
            params.motor_pos_ang_ini= item["value"];
        
        else if (item["name"] == "motor_pos_ang_fim") 
            params.motor_pos_ang_fim= item["value"];

        else if (item["name"] == "motor_pulsos_por_revolucao")
            params.motor_pulsos_por_revolucao= item["value"];

        else if (item["name"] == "motor_num_ciclos")
            params.motor_num_ciclos= item["value"]; 
            
        else if (item["name"] == "motor_delay_passo")
            params.motor_delay_passo= item["value"]; 

        else if (item["name"] == "motor_res_encoder")
            params.motor_res_encoder= item["value"];

        else if (item["name"] == "spi_sck")
            params.spi_sck= item["value"];

        else if (item["name"] == "spi_num_bits")
            params.spi_num_bits= item["value"];   

        else if (item["name"] == "spi_device")
            params.spi_device= item["value"]; 

        else{
            std::cout << "[Erro] Campo " << item["name"] << " não encontrado no arquivo: "<< path << std::endl;
            return 0; // Não encontrado
        }        
    }
    return 1;
} 
        

// Apenas habilita o PWM que controla a potencia do laser.
// O PWM é enviado a placa que converte PWM para tensão:
bool ativaLinhaLaser(PWM &pwm){
    // Verifica se o PWM da linha laser já está ativo :
    if (!pwm.getStatus()){
        // Tenta ativar o PWM que controla a linha laser:
        if (!pwm.enable())           {
            return false;
        }    
    }
    return true;
}


// APenas desativa o pwm da linha laser:
void desativaLinhaLaser(PWM &pwm){
    if (pwm.getStatus()){
        pwm.disable(); 
    }
    else{
        std::cout<<" PWM laser já está desativado." << std::endl;
    }
}


// Função que efetua a varredura de trinagulação:
void varreduraTriangulacao(gpiod_line *line_to_pulso, gpiod_line *line_to_dir, SPIDevice *spi, PARAMETROS_GERAIS &params, LightController &ledLight, PWM &pwm){
    std::cout << "Efetuando varredura laser" << std::endl;

    // Instancia o motor_passo passsando como paramentro o ponteiro spi:
    Motor motor_passo(spi);

    // Le o arquivo "params_triagulacao.json" para atualizar os paramentros usados para tratar o controle do motor.
    // Isto possibilita alterar os paramentros em tempo de execução.
    if (!lerParametrosTriangulcaoJson(params)){
        std::cout << "[Erro] Não foi possível abrir arquivo .json" << std::endl;
        return;
    }

    // Atualiza os atributos do objeto "motor_paso" para controle do motor, pois o arquivo "params_triangulacao.json" pode ter sido alterado: 
    motor_passo.setPulsoPorRevolucao(params.motor_pulsos_por_revolucao);
    motor_passo.setResolucaoEncoder(params.motor_res_encoder);

    // Posiciona o motor no angulo inicial, conforme definido no parametro "params.motor_pos_ang_ini".
    // A varredura com o laser será efetuad apenas após este posicionamento:
    motor_passo.irParaPosicaoInicial(line_to_pulso, line_to_dir, params.motor_pos_ang_ini);

    // Converte o "angulo" da posição final, dado em graus, para passos do encoder, valor entre 0 e 16383:
    int16_t pos_final= static_cast<int16_t>((params.motor_pos_ang_fim*params.motor_res_encoder)/360.0);

    // Uma vez posicionado o motor no agnulo onicial, o PWM do laser é ativado.
    // Este sinal pwm é enviado ao conversor PWM-Tensão, pois o que controla a potencia do laser é a tensão:
    if (!ativaLinhaLaser(pwm)){
        std::cout<<" [Atenção] NÂO foi possível ativa PWM -> Controla potencia da linha laser."<< std::endl;
        return;   
    }

    int ctVarredura= 0;
    // Loop de malha fechada para incrementar o motor, passo-a-passo, até a posição final definida em :
    while(true){
        ctVarredura++;
        motor_passo.lerEncoder();
        uint16_t pos_atual_encoder= motor_passo.getPositionEncoder();
        
        // Calcula a distancai entre as posições atual e a posição desejada:
        int32_t delta_pos= pos_final - pos_atual_encoder;

        // Determina o caminho mais curto para reposicionar o motor:
        if (delta_pos> (params.motor_res_encoder/2)) {
            delta_pos -= params.motor_res_encoder;
        } 
        else if (delta_pos< -(params.motor_res_encoder/2)) {
            delta_pos += params.motor_res_encoder;
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
        int delay= params.motor_delay_passo;
        motor_passo.darPasso(line_to_pulso, line_to_dir, direcao, delay);        
    }

    // Desativa o pwm da linha laser:
    desativaLinhaLaser(pwm);
    std::cout << "Varredura finalizada. Total de passo= "<< ctVarredura << std::endl;
    std::cout << std::endl;
}



// Funcção principal:
int main(int argc, char *argv[]) {
    // Rotina que limpa o terminal:
    limparTela();

    // Carrgea os parametros gerais definidos na struct "PARAMETROS_GERAIS" do header "parametros.h":
    PARAMETROS_GERAIS parametros_gerais;


    //********************************************************************************************/
    //****************************** Configuração GPIO da Jetson *******************************/
    //********************************************************************************************/

    // Declara o ponteiro do chip aqui para poder ser fechado dentro do main:
    struct gpiod_chip *chip = nullptr;

    // Isntancia um objeto para acessar a cofniguração do GPIO da Jetson. A classe está declarada no header "controlJetson.h":
    configJetson configuraGPIO_Jetson;

    // Chama o método get() para capturar as informações dos pinos e lines "ativos" da Jetson:    
    auto activePins= configuraGPIO_Jetson.getPinos();
    
    // Separa as informações dos pinos por função:
    int pin_PiscaLed= activePins.header_pin_IO_E;
    int pin_TriggerEventCam= activePins.header_pin_IO_C;
    int pin_TriggerCam= activePins.header_pin_IO_D;
    //int pin_ControlLaser= activePins.header_pinH;

    // Chama o método de configuração "configuraGPIO_Jetson.configura_GPIO_Jetson(&chip)" para inicialização do barramento GPIO da Jetson.
    // Ela retorna uma struct contendo ponteiros com os endereços de cada linha do GPIO da Jetson para controle pelo Kernell.
    // Através desses endereços que os pinos de IO são diretamente manipulados, por exemplo, mudanças de nivel lógico.
    GPIO_Lines gpios_actives= configuraGPIO_Jetson.configura_GPIO_Jetson(&chip);

    // Por segurança, verifica se as linhas "gpios_actives" foram ativadas, configuradas ok, caso contrario o programa é abortado para evitar falhas de hardware:
    if (!confirma_gpios_actives(gpios_actives, chip))
        return 1; 


    // ***********************************************************************************************/
    // ******************************** Configuraçaõ do PWM ******************************************/
    // ***********************************************************************************************/    
    // Configuração do PWM direta via Sysfs (Linux Kernel), que utiliza a interface de arquivos virtuais do 
    // sistema operacional para manipular registradores de hardware nativos. Este método elimina 
    // dependências externas, garantindo que o sinal de PWM seja gerado de forma estável por hardware 
    // independente, sem sobrecarga da CPU.

    // Define se está usando o led de potencia LT2PR da Opto Engineering
    //bool useLed_LT2PR= true;

    // Instanciando os objetos PWM com a classe PWM:
    PWM pwm_BlinkLight(parametros_gerais.periodo_PWM_A, parametros_gerais.dutyCycle_PWM_A, parametros_gerais.channelToExport_A, "Canal A");
    PWM pwm_PowerLight(parametros_gerais.periodo_PWM_B, parametros_gerais.dutyCycle_PWM_B, parametros_gerais.channelToExport_B, "Canal B");



    //********************************************************************************************/
    //********** Configuração da camera convencional FLIR (Model BlackFly BFS-U3-04S2M) **********/
    //********************************************************************************************/
    // Instancia um objeto do tipo COnvCamera: 
    ConvCamera* convCam_01 = nullptr;

    // Instanciar o Singleton: é uma instância única do sistema que gerencia a comunicação com o hardware para acessar a camera.
    // Carrega a Camada de Transporte: Inicializa os drivers e protocolos (USB3, GigE) necessários para detectar e conversar com as câmeras.
    // Ponto de Entrada: É o objeto obrigatório para listar dispositivos e gerenciar o ciclo de vida da SDK.
    Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();

    // Varredura das portas: Escaneia as interfaces (USB/GigE) em busca de câmeras conectadas.
    // Cria uma lista contendo referências (objetos CameraPtr) para todos os dispositivos encontrados.
    // Permite localizar uma câmera específica, por exemplo pelo nº de Serie para começar a operá-la.
    Spinnaker::CameraList camList = system->GetCameras();

    // Instancia o  objeto da camera convencional apenas se estiver habilitado este procedimento.
    if (parametros_gerais.useCamera_Conv){

        // Busca Seletiva: Percorre a lista de câmeras detectadas procurando o identificador único, Serial Number, definido nos parametros_gerais.
        // Verifica se a câmera desejada está conectada e disponível antes de iniciar a operação.
        Spinnaker::CameraPtr pCamBase = camList.GetBySerial(parametros_gerais.serialNumber_conv_cam_01);

        // Testa se a camera com o referido nº de séri foi encontrada:
        if (!pCamBase.IsValid()) {
            std::cerr << "Câmera não detectada!" << std::endl;
            return -1;
        }

        // Conversão de Tipo: Transforma o ponteiro genérico da SDK, Spinnaker::Camera*, no tipo definido pela minha classe ConvCamera: 
        // Permite acesso aos métodos e atributos personalizados especificos da classe ConvCamera e também da classe original da FLIR, que foi herdada.
        // Desta forma, a classe ConmvCamera terá acesso a toda as funções e metodos da classe Spinnaker::Camera:
        convCam_01 = static_cast<ConvCamera*>(pCamBase.get());
        
        // Se ok, será exibida a configuraçã da camera convencional 
        if (convCam_01){
            convCam_01->Init();
            std::cout<< std::endl;
            std::cout<< "*** Câmera convencional: ***"<< std::endl;
            convCam_01->exibir_modelo_camera();
            std::cout<< std::endl;
            convCam_01->DeInit();
        }
    }    


    //********************************************************************************************/
    //*************************** Configuração da SPI ********************************************/
    //********************************************************************************************/
    // Instancia um objeto da classe SPIDevice:
    SPIDevice spi;

    // Carregar os parametros para controle do motor lendo o arquivo "params_triagulacao.json":
    // Embora esses parametros sejam gerados em tempo de compilação diretamente na struct "parametros_gerais" declarada em "parametros.h",
    // a leitura do .json possibilita alterar o valor desses paramentros em tempo de execução, basta editar o .json e chamar a função para alterar essses valores.
    if (!lerParametrosTriangulcaoJson(parametros_gerais)){
        std::cout << "[Erro] Não foi possível abrir arquivo .json" << std::endl;
        return 1;
    }
    
    // Metodo que abre e iniciliza a interface spi com os parametros definidos em "parametros.h": 
    if (!spi.inicializa(parametros_gerais.spi_device, parametros_gerais.spi_mode, parametros_gerais.spi_sck, parametros_gerais.spi_num_bits)){
        std::cout << "[Erro] não foi possivel inicair a interface SPI." << std::endl;
        return 1;
    }

    // std::cin.get();


    //********************************************************************************************/
    //*************************** Configuração da camera de eventos ******************************/
    //********************************************************************************************/

    // Se não detectar camera de eventos no barramente USB, o programa é abortado para evitar falhas de hardware:
    if (!detectaCamerasConectadas()){
        return 1;
    }

    // Instancia o objeto event_cam_xx para operar com a camera de eventos:
    // Esta "Classe EventCamera" foi criada para tratar dos objeos, atributops e métodos do SDK Metavision: 
    // O construtor recebe um parametro booleano do tipo "argumento padrão", só é passado quando for necessário,
    // Instanciação dos objetos que controlam o hardware das câmeras de eventos:
    EventCamera event_cam= EventCamera(parametros_gerais.serialNumber_event_cam3, "Event Mono", true);


    // Configura alguns parametros iniciais para cada câmera de eventos, como o bias, trigger e sincronismo de hardware, 
    // usando as funções da classe EventCamera, que por sua vez utilizam a API do SDK Metavision:
    // COnfigura, carrega, os biases na camera de eventos definidos em setings.json:
    if (!event_cam.setBias(parametros_gerais)){
        std::cerr << "Câmera de eventos não detectada!" << std::endl;
        return -1;
    }

    // Imporante!!! Esta função habilita o Trigger via HAL da API:
    event_cam.enableHardwareTrigger();

    // Captura as dimensões da camera de eventos para configurar a visualização no dashboard:
    int cam_w = event_cam.getWidth();
    int cam_h = event_cam.getHeight();

    // Instanci a struct que contem os parâmetros para configurar o construtor da visualização, 
    // que é o objeto responsável por gerenciar a visualização dos frames das câmeras de eventos e do menu no dashboard:
    configFrameGenerator configFrameGen;

    // Função que configura o frameGenerator, responsável por montar os frames de eventos que serão exibdos no canvas, 
    // janela de visualização via OpenCV:
    configuraFrameGenerator(configFrameGen, event_cam);

    // Dispara o funcionamenoe das câmeras de eventos.
    // A partir deste ponto, as câmeras de eventos começam a capturar e gerar frames, 
    // que são processados pelos geradores de frames e exibidos no dashboard em tempo real.
    // A contagem do laço for é invertida porque a câmera SLAVE deve ser iniciada antes da MASTER para 
    // garantir que ela esteja pronta para receber os sinais de sincronismo e trigger da master.
    event_cam.callStart();

    
    // Instacia objeto "ledLight" do tipo "LighControler", definido em "controlIO.h". 
    // Esta classe dfoi criada apenas para controlar o estado do Led, laser, luz estruturada:
    LightController ledLight;
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
                case '1':{
                        // Efetua a leitura dos biases da camera de eventos:
                        event_cam.readCameraBiases();
                        break;
                }
                
                case '2':
                        // Chama função para configurar, enviar, os biases à camera de eventos:                    
                        event_cam.readCameraBiases();
                        break;

                case '3':
                        {
                            // Usa-se uma thread com função lambda, onde:
                            // [&] = Captura: ela captura todas as variáveis e métodos, por referencia, no escopo de main(), pois ela utilzia várias funções e variáveis;
                            // () = Parâmetros: Sem nenhum parâmetro como argumento;
                            // {} = Corpo: onde são chamadas as funções.:
                            std::thread t([&]() { 
                                for (int ctCiclo=0;  ctCiclo < parametros_gerais.numero_ciclos_trigger; ctCiclo++){                           
                                    // Primeira ativa a projeção da luz estruturada:
                                    ativaLedLight(ledLight, pwm_BlinkLight, pwm_PowerLight);
                                    // Chama a gravação do dados da câmera de eventos, que é feita em paralelo, ou seja, enquanto a câmera de eventos está gravando os 
                                    // dados no arquivo .raw, o programa continua executando as próximas linhas de código, sem esperar a finalização da gravação.  
                                    saveData_Mono_TriggerHW(event_cam, gpios_actives.triggerEventCam, parametros_gerais);
                                    // Por ultimo, desativa o projeção de luz estruturada:
                                    desativaLedLight(ledLight, pwm_BlinkLight, pwm_PowerLight);
                                }
                            });

                            // Desacoplar a thread para que o viewer não trave
                            t.detach();
                            break;
                        } 

                case '4':
                        {
                            if (!ledLight.getRunning())
                                ativaLedLight(ledLight, pwm_BlinkLight, pwm_PowerLight);
                            else
                                desativaLedLight(ledLight, pwm_BlinkLight, pwm_PowerLight);                                             
                            break;                        
                        }  
                        
                case '5':
                        {
                            // Captura uma imagem pela câmera convencional:
                            if (convCam_01){
                                convCam_01->Init();
                                convCam_01->capturarImagem();
                                convCam_01->DeInit();
                            }
                            else
                                std::cout << "Câmera convencional não instanciada." << std::endl;
                            break;                        
                        }   
                        
                case '6':
                        {
                            std::thread thread_motor([&]() {                         
                                varreduraTriangulacao(gpios_actives.controlMotor_Pulso, gpios_actives.controlMotor_Dir, &spi, parametros_gerais, ledLight, pwm_PowerLight); 
                            });
                            thread_motor.detach();    
                            break;  
                    }
                             
                case '.':
                case '>': // Incrementa o pwm que controla o tempo de atuação do Led/laser, duração do blink:
                        {
                           if (parametros_gerais.useLed_LT2PR)
                                incrementaPWM(pwm_BlinkLight, "Duty Cycle Blink", parametros_gerais.useLed_LT2PR);
                           else
                                std::cout << " [Atenção!] Use (+) para incrementar o Duty Cicle do laser.\n";   
                          break;    
                        }    
                case ',':    
                case '<': // Decrementa o pwm que controla o tempo de atuação do Led/laser, duração do blink:
                        {
                            if (parametros_gerais.useLed_LT2PR)
                                decrementaPWM(pwm_BlinkLight, "Duty Cycle Blink");
                            else
                                std::cout << " [Atenção!] Use (-) para decrementar o Duty Cicle do laser.\n";                        
                            break;
                        }                    

                case 171: // tecla + do bloco numerico        
                case '+':
                case '=': // Incrementa o pwm que controla a tensão analogica do Led/laser (0 a 10V) ou laser (o a 5V):
                        incrementaPWM(pwm_PowerLight, "Duty Cycle da tensão", false);                                 
                        break;

                case 173: // codigo - do bloco numerico        
                case '-':
                case '_': // Decremento o pwm que controla a tensão analogica do Led/laser (0 a 10V) ou laser (o a 5V):
                        decrementaPWM(pwm_PowerLight, "Duty Cycle da tensão");
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


    // Fecha a câmera:  
    event_cam.stop();

    // Fecha a SPI:
    spi.finaliza();

    // Para o gerador de frame:
    if (event_cam.getGenerator()) {
        event_cam.getGenerator()->stop();
    }

    // Fecha todas as janelas:    
    cv::destroyAllWindows();

    // Desabilita o pwm
    pwm_BlinkLight.disable();
    pwm_PowerLight.disable();

    return 0;
}