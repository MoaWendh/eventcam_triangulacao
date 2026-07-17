# Projeto Voris: Sistema de Triangulação Laser com Câmera de Eventos

**Autor:** Moacir Wendhausen  
**Data:** 01/07/2026  

---

## 💻 Ambiente de Desenvolvimento e Hardware

* **Processamento:** Nvidia Jetson Orin Nano
* **Sistema Operacional:** Ubuntu 22.04
* **Câmera de Eventos:** SilkyEvCam (Century Arks)
* **Bibliotecas base:** OpenCV 4.8.0
* **Driver/SDK:** Metavision 5.1.1

---

## ⚙️ Funcionalidades Principais

* **Varredura Óptica:** Projeção de linha laser com varredura espacial controlada por um espelho plano acoplado ao eixo de um motor de passo.
* **Controle de Potência:** Modulação da intensidade do laser através de sinal PWM.
* **Processamento Assíncrono:** Arquitetura baseada em *multithreading*, isolando o controle físico da varredura de triangulação da renderização visual. Isso evita o congelamento de quadros durante a atuação do motor.
* **Interface Gráfica (GUI):** Renderização da matriz de eventos em tempo real e menu de controle de parâmetros desenvolvidos nativamente sobre o Canvas do OpenCV.
* **Aquisição e Gravação de Dados:** 
  * Exportação bruta de eventos para arquivos de dados binários formato `.dat`.
  * Sincronização espacial: Durante a varredura, o software mapeia e registra um arquivo `.dat` exclusivo para cada incremento (posição angular) da linha laser.

---

## 📷 Especificações do Sensor (SilkyEvCam)

| Parâmetro | Configuração / Valor |
| :--- | :--- |
| **Fabricante / Integrador** | CenturyArks |
| **Modelo do Sensor** | Gen3.1 |
| **Interface de Conexão** | USB |
| **Formato de Codificação (Atual e Disp.)** | EVT3 |
| **Versão do Sistema** | 4.2.0 |
| **Versão do Firmware (Release)** | 3.9.0-C |
| **Data de Build do Firmware** | Sun Oct 30 23:37:15 2022 |
| **Velocidade do Firmware** | 5000 |
| **Número de Série** | 00000680 |