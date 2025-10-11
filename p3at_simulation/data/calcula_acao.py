import numpy as np

def calcula_acao(Q_table, estado_discreto, epsilon):
    """
    Escolhe a próxima ação usando a política epsilon-greedy.
    
    Parâmetros:
        Q_table: np.ndarray -> tabela Q [num_x, num_y, num_acoes]
        estado_discreto: tuple/list -> (idx_x, idx_y) estado atual
        epsilon: float -> taxa de exploração (0-1)

    Retorna:
        acao_escolhida: int -> índice da ação escolhida
    """
    num_acoes = Q_table.shape[2]

    if np.random.rand() < epsilon:
        # Exploração: ação aleatória
        acao_escolhida = np.random.randint(0, num_acoes)
    else:
        # Explotação: ação com maior valor na Q-table
        idx_x, idx_y = estado_discreto
        acao_escolhida = np.argmax(Q_table[idx_x, idx_y, :])
    
    return acao_escolhida
