"""
converter_npy.py
=================
Converte uma IMAGEM de labirinto (PNG/JPG — ex.: um mapa desenhado ou uma
figura salva pelo plot_maze/generate_maze) em uma matriz 32×32 salva em .npy,
no mesmo formato consumido por generate_maze.py / qlearning_visualizer.py
(0 = caminho livre, 1 = parede).

Pipeline (OpenCV):
  1. Carrega a imagem e converte para tons de cinza.
  2. Binariza (THRESH_BINARY_INV) para destacar linhas/paredes.
  3. Isola a região do labirinto pelo maior contorno externo.
  4. Divide o recorte numa grade 32×32 e amostra o pixel CENTRAL de cada
     célula: pixel escuro (< 50) → parede (1); claro → livre (0). O limiar
     50 garante que marcadores coloridos de Início/Meta sejam lidos como
     caminho livre.
  5. Salva a matriz em .npy.

Uso: edite as duas linhas finais (imagem de entrada / arquivo de saída) e
rode `python converter_npy.py`. Requer: pip install opencv-python numpy
"""

import cv2
import numpy as np

def converter_imagem_para_npy(caminho_imagem, caminho_saida, tamanho_grade=32):
    # 1. Carregar a imagem
    img = cv2.imread(caminho_imagem)
    if img is None:
        print("Erro: Não foi possível carregar a imagem. Verifique o caminho.")
        return

    # Converter para tons de cinza
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 2. Binarizar a imagem (Fundo vira preto, linhas/textos viram brancos)
    _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)

    # 3. Encontrar os contornos para isolar a área do labirinto
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # A grade principal do labirinto será o maior contorno da imagem
    maior_contorno = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(maior_contorno)

    # 4. Recortar apenas a região do labirinto
    labirinto_roi = gray[y:y+h, x:x+w]

    # 5. Criar matriz vazia para armazenar os dados (32x32)
    matriz_labirinto = np.zeros((tamanho_grade, tamanho_grade), dtype=np.int8)

    # Calcular o tamanho de cada célula na imagem recortada
    largura_celula = w / tamanho_grade
    altura_celula = h / tamanho_grade

    # 6. Varrer as células para identificar caminhos (0) e paredes (1)
    for linha in range(tamanho_grade):
        for coluna in range(tamanho_grade):
            # Encontrar as coordenadas do pixel central da célula atual
            centro_x = int((coluna * largura_celula) + (largura_celula / 2))
            centro_y = int((linha * altura_celula) + (altura_celula / 2))

            # Ler a cor do pixel (valores próximos de 0 são pretos)
            valor_pixel = labirinto_roi[centro_y, centro_x]

            # O limiar 50 garante que o Início (verde) e a Meta (vermelho) sejam lidos como caminho (0)
            if valor_pixel < 50:
                matriz_labirinto[linha, coluna] = 1 # Parede
            else:
                matriz_labirinto[linha, coluna] = 0 # Caminho livre

    # 7. Salvar o array no formato .npy
    np.save(caminho_saida, matriz_labirinto)
    print(f"Sucesso! Matriz salva em: {caminho_saida}")

# Executar a conversão
converter_imagem_para_npy('fechado_pior_caso.png', 'fechado_pior_caso.npy')