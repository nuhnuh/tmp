# Summary

xml snippet on how to parse xml from i3code


# xml schema

    Circuito
      Rutas
        Ruta
          Tramos
            Tramo
              MarchaAdelante
              MarchaAtras
              PuntoFin
                Acciones
                  Accion
              Puntos
                Punto
                  X
                  Y
                  S


# Dudas:

- hay algo raro en la nomenclatura: se usa el concepto punto para definir inicio y fin de tramo y para definir el tramo

    PuntoInicio.Id or PuntoFin.Id not in enumerate(Puntos)?

- PuntoFin/Acciones/Accion ¿Que acciones son posibles además de Stop?

- Direccion de la marcha: MarchaAdelante=True and MarchaAtras=True?

- En el ejemplo 1 / Ruta larga se hace un cambio de sentido peligroso en el punto 11


# src code

    import xml.etree.ElementTree as ET
    mytree = ET.parse('data/20211026/circuitoNavegacion_emplo1.xml')
    print(mytree.getroot())

    assert len([rutas.tag for rutas in mytree.findall('Rutas')]) == 1

    [ruta.findall('Nombre')[0].text for ruta in mytree.findall('Rutas')[0].findall('Ruta')]
    [ruta.findall('Nombre')[0].text for ruta in mytree.findall('Rutas')[0].iter('Ruta')]

    # numbre de las rutas
    [ruta.find('Nombre').text for ruta in mytree.find('Rutas').iter('Ruta')]

    # numero de tramos en Ruta 'Ruta larga'
    [len(ruta.find('Tramos').findall('Tramo')) for ruta in mytree.find('Rutas').iter('Ruta') if ruta.find('Nombre').text == 'Ruta larga']

    # Ruta['Ruta larga'] / Tramo['Punto (1) al punto (8)']
    ruta = [ruta for ruta in mytree.find('Rutas').iter('Ruta') if ruta.find('Nombre').text == 'Ruta larga'][0]
    tramos = ruta.find('Tramos').findall('Tramo')
    for tramo in tramos:
      print('Tramo: ' + tramo.find('Nombre').text)
      print('  forward: ' + tramo.find('MarchaAdelante').text + ', numero de puntos: ' + str(len(tramo.find('Puntos').findall('Punto'))))
      #for punto in tramo.find('Puntos').iter('Punto'):
      #  print('  ' + punto.find('X').text)
    > Tramo: Punto (1) al punto (8)
    >   forward: true, numero de puntos: 160
    > Tramo: Punto (8) al punto (9)
    >   forward: false, numero de puntos: 80
    > Tramo: Punto (9) al punto (10)
    >   forward: false, numero de puntos: 102
    > Tramo: Punto (10) al punto (11)
    >   forward: false, numero de puntos: 127
    > Tramo: Punto (11) al punto (7)
    >   forward: true, numero de puntos: 46
    > Tramo: Punto (7) al punto (6)
    >   forward: true, numero de puntos: 101
    > Tramo: Punto (6) al punto (5)
    >   forward: true, numero de puntos: 101
    > Tramo: Punto (5) al punto (4)
    >   forward: true, numero de puntos: 159
    > Tramo: Punto (4) al punto (3)
    >   forward: true, numero de puntos: 101
    > Tramo: Punto (3) al punto (2)
    >   forward: true, numero de puntos: 100
    > Tramo: Punto (2) al punto (1)
    >   forward: true, numero de puntos: 100
