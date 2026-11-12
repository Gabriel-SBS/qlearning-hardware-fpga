"""
edit_maze.py
=============
Editor gráfico interativo de labirintos .npy (matriz 32×32, 0 = livre,
1 = parede) — útil para ajustar manualmente mapas gerados pelo
generate_maze.py ou convertidos pelo converter_npy.py.

Controles (na janela Matplotlib):
  * Clique em uma célula      → alterna parede ↔ caminho livre
  * Tecla 'S'                 → salva as alterações NO MESMO arquivo

Uso: edite a última linha (caminho do .npy) e rode `python edit_maze.py`.
Requer: pip install numpy matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt

def editar_labirinto(caminho_arquivo):
    # 1. Carregar a matriz do labirinto
    try:
        labirinto = np.load(caminho_arquivo)
    except FileNotFoundError:
        print(f"Erro: O arquivo '{caminho_arquivo}' não foi encontrado.")
        return

    # Configurar a janela de exibição
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title('Editor de Labirinto')
    
    # Adicionar título com as instruções
    ax.set_title("Botão Esquerdo/Direito: Adicionar/Remover Parede\nTecle 'S' para salvar as alterações.")

    # 2. Exibir o labirinto (0 vira branco, 1 vira preto)
    cmap = plt.cm.binary
    im = ax.imshow(labirinto, cmap=cmap, vmin=0, vmax=1)

    # Adicionar uma grade sutil para facilitar a visualização de cada célula
    ax.set_xticks(np.arange(-0.5, labirinto.shape[1], 1), minor=True)
    ax.set_yticks(np.arange(-0.5, labirinto.shape[0], 1), minor=True)
    ax.grid(which="minor", color="gray", linestyle='-', linewidth=0.5)
    ax.set_xticks([]) # Oculta os números do eixo X
    ax.set_yticks([]) # Oculta os números do eixo Y

    # 3. Função para capturar o clique do mouse
    def onclick(event):
        # Ignorar cliques fora do gráfico
        if event.inaxes != ax:
            return
            
        # Converter as coordenadas do clique para os índices da matriz
        coluna = int(round(event.xdata))
        linha = int(round(event.ydata))

        # Garantir que o clique esteja dentro dos limites da matriz
        if 0 <= linha < labirinto.shape[0] and 0 <= coluna < labirinto.shape[1]:
            # Alternar o valor: se for 1 vira 0, se for 0 vira 1
            labirinto[linha, coluna] = 1 - labirinto[linha, coluna]
            
            # Atualizar a imagem na tela
            im.set_data(labirinto)
            fig.canvas.draw()

    # 4. Função para capturar atalhos de teclado (Salvar)
    def onpress(event):
        if event.key.lower() == 's':
            np.save(caminho_arquivo, labirinto)
            print(f"--> Sucesso! Labirinto atualizado e salvo em: {caminho_arquivo}")

    # Conectar os eventos do mouse e teclado à figura
    fig.canvas.mpl_connect('button_press_event', onclick)
    fig.canvas.mpl_connect('key_press_event', onpress)

    # Iniciar a interface
    print("Editor aberto. Edite clicando nas células. Pressione 'S' na janela do gráfico para salvar.")
    plt.show()

# Executar o editor apontando para o arquivo gerado
editar_labirinto('fechado.npy')