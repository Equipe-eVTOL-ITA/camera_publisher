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

Isso ja aconteceu tres vezes neste workspace. Na fase 3 o config pedia
`<nome>/image/compressed` enquanto o `webcam` publicava `<nome>_camera/...`, e o
drone girou procurando uma mao que ninguem enxergava. Na fase 1, o
`flight.yaml` pedia `/vertical_camera/image/compressed` e o launch subia o
`webcam` -- o mesmo defeito, esperando o primeiro voo para aparecer.

A terceira nao foi de FORMATO, e sim de NOME, e por isso esta funcao sozinha nao
a pegou: o drone rodava o `roi_stream` (`camera_name: 'gesto'`) enquanto o
`flight.yaml` da fase 3 mandava o detector assinar `/frontal_camera/compressed`.
Os dois lados usavam a convencao certa, com camera_name diferente. O drone
decolou, girou em SEARCH HAND ate a bateria, e o unico sintoma visivel foi a
imagem de debug do detector nunca aparecer -- porque ela e publicada dentro do
callback do quadro, que nunca rodou.

Com a funcao aqui, o FORMATO sai de um lugar so, e o `test_topicos.py` confere
que todo publicador a usa. O NOME continua sendo acordo entre config e config:
confira com `ros2 topic info -v <topico>` antes de armar, e desconfie de
"Publisher count: 0".
"""

from __future__ import annotations


# >>> CONTRATO topicos.camera
# O nome do topico de uma camera e:
#
#     <papel>_camera/compressed        SEM /image/ NO MEIO
#     <papel>_camera/raw
#
# `papel` e a FUNCAO, nao o hardware: vertical, frontal, horizontal, gesto.
# Trocar a camera nao pode trocar o topico.
#
# POR QUE ISTO ESTA ESCRITO EM LETRA GRANDE
#
# Ja houve tres formas em uso ao mesmo tempo -- /camera/image/compressed,
# /vertical_camera/image/compressed, /vertical_camera/compressed -- e trocar de
# camera trocava o topico enquanto o detector do outro lado seguia assinando o
# antigo. Ele sobe, nao reclama, e nunca recebe quadro. A missao voa CEGA e o
# log nao diz por que.
#
# Isso ja aconteceu TRES VEZES neste workspace.
#
# Antes de armar:  ros2 topic info -v <topico>
# e desconfie de "Publisher count: 0".
# <<< CONTRATO

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
