// ***************************************************************************************************************************************************************
// @ Autor: Moacir Wendhausen    
// @ Projeto: VORIS
// @ Data: 10/01/2026
// @ Principal função: 
//  - Controlar do processo de aquisição de eventos de um sistema de triangulação baseado em camera de eventos e projeção de linha laser. 
//
// @ Demais funcionalidades:
//  1- Motor de paso: controle de posicionamento, passo, do motor baseado em malha fechada, a partir de um posição angular final especificada.
//  2- Encoder absoluto: O encoder é controlado através de uma interface SPI da Jetson Orin Nano, uado para saber a posição motor bem como para posicioná-lo.    
//  3- Varredura Óptica com lasar de linha: Projeção de linha laser com varredura espacial controlada por um espelho plano acoplado ao eixo do motor de passo.
//  4- Controle de Potência: Modulação da intensidade do laser através de sinal PWM.
//  5- Processamento Assíncrono: Arquitetura baseada em multithreading, isolando o controle físico da varredura de triangulação da renderização visual. 
//     Isso evita o congelamento de quadros durante a atuação do motor.
//  6- Interface Gráfica (GUI): Renderização da matriz de eventos em tempo real e menu de controle de parâmetros desenvolvidos nativamente sobre o Canvas do OpenCV.
//  7- Aquisição e Gravação de Dados: Exportação bruta de eventos para arquivos de dados binários formato `.dat`.
//  8- Sincronização espacial: Durante a varredura, o software mapeia e registra um arquivo `.dat` exclusivo para cada incremento, posição angular, da linha laser.
//
// @ Recursos de SW:
//  - Sistema Linux Ubuntu versão 22.04
//  - SDK Metavisiona versão 5.1.1 da Prophesee
//  - Plugin da camera de eventos da Century Arks versão 5.1.1  
//  - OpenCV versão 4.8.0
//  - IDE desenvolvimento Visual Studio Code (Vscode) 1.129.1
//
// @ Recursos de HW:
//  - CPU NVidia modelo Jetson Orin Nano
//  - Câmera de eventos SilkyEvCam da Century Arks
//  - Motor de passo 
//  - Fonte laser com lente para projeção de linha
//  - Encoder
//  - Modularo PWM
//  - Driver para controle do motor  
// ***************************************************************************************************************************************************************
#include <iostream>

#include "control_triangulacao.h"


// Função principal:
int main(int argc, char *argv[]) {
    // Comando para limpar a tela:
    std::cout << "\033[2J\033[H" << std::flush;

    // Instancia a classe que "ctrl_triangulacao" da classe ControlTriangulation, que contém todos os atributos e metodos necessários para acessar 
    // e controlar todo o HW do sistema de triangulação, incluindo: 
    //  - motor,
    //  - encoder, 
    //  - camera de eventos, 
    //  - laser, 
    //  - GPIO da Jetson, 
    //  - PWM da Jetson,
    //  - SPI da Jetson. 
    ControlTriangulacao ctrl_triangulacao;

    if (ctrl_triangulacao.inicializaHardware()) {
        ctrl_triangulacao.executaLoopPrincipal();
    }

    // Método que finalização o programa de trinagulação::
    ctrl_triangulacao.finalizaHardware(); 

    return 0;
}