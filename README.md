### Sistema de triangulação laser com câmera de eventos
Projeto: Voris  
Data: 01/07/2026  
Autor: Moacir Wendhausen  
SDK Metavision: 5.1.1
Câmera de eventos: SilkyEvCam da Century Arks
Ubuntu 22.04
Hardware: CPU Nvidia Jetosn Orin Nano
OpenCV: versão 4.8.0
---
### Características da SilkyEvCam  

    Available Data Encoding Formats                   EVT3
    Connection                                        USB
    Current Data Encoding Format                      EVT3
    FW Build Date                                     Sun Oct 30 23:37:15 2022
    FW Release Version                                3.9.0-C
    FW Speed                                          5000
    Integrator                                        CenturyArks
    Sensor Name                                       Gen3.1
    Serial                                            00000680
    System Version                                    4.2.0

------

**Principais características do programa:**  
  
1- Projeção de linha laser com varredura controlada por um espelho plano acoplado ao eixo do motor de passo.

2- Potência do laser controlada por PWM

3- Exibição dos eventos e Menu de opções baseados no Canvas OpenCV

4- Os dados de eventos são registrados em arquivo de dados binários do tipo .dat

5- Durante a varredura, para cada posição da linha laser é registrado um arquivo de dados de evetos .dat. 

6- Varredura da trinagulação e visualização dos eventos são executados em threads separadas.
