"""
A convencao de nome de topico das cameras, escrita UMA vez.

    <camera_name>_camera/compressed
    <camera_name>_camera/raw

SEM `/image/` NO MEIO. Nenhum publicador deste pacote o usa, e nenhum config
deve escreve-lo.

POR QUE ISTO EXISTE

Os quatro publicadores tinham cada um a sua ideia do nome:

    webcam      <nome>_camera/compressed        <- a convencao
    raspicam    vertical_camera/image/compressed
    jetson      camera/image/compressed
    oak         camera/image/compressed

Tres formas diferentes, e nenhum erro em lugar nenhum. Trocar de camera trocava
o topico, e o detector do outro lado continuava assinando o antigo: ele sobe,
nao reclama, e nunca recebe quadro. A missao voa CEGA e o log nao diz por que.

Isso ja aconteceu duas vezes neste workspace. Na fase 3 o config pedia
`<nome>/image/compressed` enquanto o `webcam` publicava `<nome>_camera/...`, e o
drone girou procurando uma mao que ninguem enxergava. Na fase 1, o
`flight.yaml` pedia `/vertical_camera/image/compressed` e o launch subia o
`webcam` -- o mesmo defeito, esperando o primeiro voo para aparecer.

Com a funcao aqui, o nome sai de um lugar so, e o `test_topicos.py` conferindo
que todo publicador a usa.
"""

from __future__ import annotations


def nome_do_topico(camera_name: str, comprimido: bool) -> str:
    """
    O topico em que uma camera chamada `camera_name` publica.

    `camera_name` e o papel da camera no drone -- `vertical`, `frontal`,
    `horizontal` --, e nao o modelo do hardware. E de proposito: trocar uma
    webcam por uma Raspberry Pi Camera na mesma posicao NAO pode mudar o topico,
    ou todo config que a consome quebra em silencio.
    """
    if not camera_name:
        raise ValueError("camera_name vazio: o topico sairia como '_camera/...'")
    return f"{camera_name}_camera/{'compressed' if comprimido else 'raw'}"
