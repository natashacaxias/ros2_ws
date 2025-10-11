import numpy as np

def calc_estados(Er, pose, destino, U, d, h, caso):
    """
    Estima o estado atual do robô pelo modelo cinemático.

    Parâmetros:
        Er: np.array [3,1] -> erro atual [erro_x, erro_y, theta]
        pose: np.array [3,1] -> pose atual do robô [x, y, theta]
        destino: np.array [2,1] -> posição alvo [x, y]
        U: np.array [2,1] -> velocidades [v, rot]
        d: float -> distância do ponto de controle à frente do robô
        h: float -> passo de integração
        caso: int -> 1 para simulação direta do erro, 2 para recalcular pose e erro

    Retorna:
        Er: np.array [3,1] -> erro atualizado
    """

    if caso == 1:
        # Modelo cinemático direto (simulação)
        B = np.array([
            [-1, Er[1,0]],
            [0, -(Er[0,0]+d)],
            [0, 1]
        ])
        Er_pt = B @ U
        for _ in range(100):
            Er[0,0] += h * Er_pt[0,0]
            Er[1,0] += h * Er_pt[1,0]
            Er[2,0] = (Er[2,0] + h * Er_pt[2,0]) % (2 * np.pi)

    elif caso == 2:
        # Atualiza pose e recalcula erro
        for _ in range(100):
            pose[0,0] += h * U[0,0] * np.cos(pose[2,0])
            pose[1,0] += h * U[0,0] * np.sin(pose[2,0])
            pose[2,0] = (pose[2,0] + h * U[1,0]) % (2 * np.pi)

        # Erro de posicionamento no frame do robô
        E = destino - pose[0:2,:]
        cs, ss = np.cos(pose[2,0]), np.sin(pose[2,0])
        Rot = np.array([[cs, ss], [-ss, cs]])
        Er[:2,0] = (Rot @ E - np.array([[d],[0]])).flatten()
        Er[2,0] = pose[2,0]

    return Er
