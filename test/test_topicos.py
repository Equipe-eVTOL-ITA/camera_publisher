"""
A convencao de topico das cameras, travada.

O QUE ESTE ARQUIVO IMPEDE

Os quatro publicadores tinham cada um a sua ideia do nome do topico -- tres
formas diferentes, e nenhum erro em lugar nenhum. Um detector que assina o nome
errado SOBE, nao reclama, e nunca recebe quadro: a missao voa cega e o log nao
diz por que.

Ja aconteceu duas vezes aqui. Na fase 3 o drone girou procurando uma mao que
ninguem enxergava. Na fase 1 o mesmo defeito estava no `flight.yaml`, esperando
o primeiro voo para aparecer.
"""

import pathlib
import re

import pytest

from camera_publisher.topicos import nome_do_topico

PACOTE = pathlib.Path(__file__).resolve().parent.parent / "camera_publisher"


def test_a_convencao_nao_tem_image_no_meio():
    assert nome_do_topico("vertical", True) == "vertical_camera/compressed"
    assert nome_do_topico("frontal", False) == "frontal_camera/raw"


def test_o_papel_da_camera_e_que_nomeia():
    """
    `vertical`, `frontal` -- e nao `webcam`, `raspicam`, `oak`.

    Trocar o hardware da camera na MESMA posicao do drone nao pode mudar o
    topico, senao todo config que a consome quebra em silencio.
    """
    assert nome_do_topico("vertical", True).startswith("vertical_")


def test_nome_vazio_e_erro():
    # Sem isto o topico sairia como '_camera/compressed', que casa com nada.
    with pytest.raises(ValueError):
        nome_do_topico("", True)


def test_nenhum_publicador_escreve_o_topico_a_mao():
    """
    A convencao mora em `topicos.py`, e so la.

    Um publicador que monta o nome sozinho volta a divergir dos outros na
    primeira vez que alguem mexer nele -- que e exatamente como as tres formas
    diferentes apareceram.
    """
    suspeito = re.compile(r"""["'][a-z_]*camera/(image/)?(compressed|raw)["']""")
    faltando = []

    for arquivo in sorted(PACOTE.glob("*.py")):
        if arquivo.name in ("topicos.py", "__init__.py"):
            continue
        texto = arquivo.read_text(encoding="utf-8")
        if "create_publisher" not in texto:
            continue

        if achado := suspeito.search(texto):
            faltando.append(f"{arquivo.name}: topico literal {achado.group(0)}")
        elif "nome_do_topico" not in texto:
            faltando.append(f"{arquivo.name}: publica sem usar nome_do_topico")

    assert not faltando, "publicadores fora da convencao:\n  " + "\n  ".join(faltando)


def test_nenhum_publicador_usa_image_no_caminho():
    """A busca direta pelo que se quer eliminar."""
    maus = [f.name for f in PACOTE.glob("*.py")
            if f.name != "topicos.py" and "image/compressed" in f.read_text()]
    assert not maus, f"ainda escrevem 'image/' no topico: {maus}"
