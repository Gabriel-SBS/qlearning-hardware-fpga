"""
plot_maze.py
=============
Plota um WALL_MAP a partir da string hexadecimal de 1024 bits no formato do
parâmetro Verilog WALL_MAP_DEFAULT (maze_env.v) — a mesma convenção do
maze_to_hex() do generate_maze.py (índice 0 da string hex = MSB = bit 1023;
bit N do vetor = célula (N // 32, N % 32)).

Uso: cole a string hex desejada na variável `hex_val` e rode
`python plot_maze.py`. Gera 'labirinto_original.png' e abre a janela.
Requer: pip install numpy matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt

def plot_original_maze():
    # A string hexadecimal exata do seu arquivo maze_env.v
    hex_val = "0E18201ECA212078C6E08C281129A904040840050900340927820586002800B20A003C6026204201006130208321046681E24120080024682655473320180B1014221388C3807B7118AD8A15004830491509340360300428A4D5021042200350D0648411C0A10209481241E2402922242004D001000540E1226044C027001A66"
    
    # Converte o hex para um binário de 1024 bits (adicionando zeros à esquerda se faltar)
    binary_val = bin(int(hex_val, 16))[2:].zfill(1024)
    
    maze = np.zeros((32, 32))
    
    # Preenche a matriz
    for r in range(32):
        for c in range(32):
            bit_idx = r * 32 + c
            # Em python, o índice 0 da string é o bit mais significativo (MSB, bit 1023)
            # O índice 1023 da string é o bit menos significativo (LSB, bit 0)
            bit_val = binary_val[1023 - bit_idx]
            maze[r, c] = int(bit_val)
            
    # Criar a imagem
    plt.figure(figsize=(10, 10))
    # cmap='binary' mapeia 0 para branco e 1 para preto
    plt.imshow(maze, cmap='binary')
    
    # Adicionar marcadores de Início e Fim
    plt.plot(0, 0, 'go', markersize=12, label='Início (0,0)') # Verde
    plt.plot(31, 31, 'ro', markersize=12, label='Meta (31,31)') # Vermelho
    
    # Configurar grelha
    plt.grid(True, which='both', color='lightgrey', linewidth=0.5)
    plt.xticks(np.arange(-0.5, 32, 1), [])
    plt.yticks(np.arange(-0.5, 32, 1), [])
    
    plt.legend(loc='upper right')
    plt.title('Labirinto Original do Q-Learning (32x32)', fontsize=16)
    
    # Salvar e mostrar
    plt.savefig('labirinto_original.png', bbox_inches='tight')
    print("Imagem 'labirinto_original.png' gerada com sucesso!")
    plt.show()

if __name__ == "__main__":
    plot_original_maze()