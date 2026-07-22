
// ***************************************************************************************************************************************************************
// @ Autor: Moacir Wendhausen    
// @ Projeto: VORIS
// @ Data: 15/05/2026
//
// Função: 
//
// Função: Este arquivo contém a implementação da classe ConvCamera, que é responsável por controlar a câmera convencional usando o SDK Spinnaker da FLIR. 
// A classe ConvCamera herda todos os métodos e atributos da classe Spinnaker::Camera, e sobrescreve os métodos de inicialização, captura de imagem, entre outros, 
// para adaptar às necessidades do projeto.
// ***************************************************************************************************************************************************************


#pragma once


#include "Spinnaker.h"
#include <string>
#include <iostream>

// Esta classe ConvCamera herda todos os métodos e atributos da classe Spinnaker::camera:
class ConvCamera : public Spinnaker::Camera {
private:
    // Apenas guarda o estado aual da camera instanciada:
    bool inicializada;

public:

    Spinnaker::GenApi::INodeMap& get_nodemap();

    ConvCamera();
    virtual ~ConvCamera();

    // Sobrescrevemos os métodos:
    void Init();
    void DeInit();

    void exibir_modelo_camera();
    std::string get_serial() const;
    bool is_ok();

    Spinnaker::ImagePtr capturarImagem();    
};