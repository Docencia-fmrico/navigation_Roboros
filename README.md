# Práctica de navegación

[![GitHub Action
Status](https://github.com/Docencia-fmrico/navigation/workflows/main/badge.svg)](https://github.com/Docencia-fmrico/navigation)


**Entrega:** Miércoles 2/3 

En la moqueta verde del laboratorio se limitará con unas paredes, y se pondrán obstáculos (cajas) dentro el viernes 25/2. No habrá cambios en el escenario desde este momento. El miércoles, al inicio de la clase se proporcionarán un conjunto de waypoints en un fichero de parámetros como este:

```
patrolling_node:
  ros__parameters:
    waypoints: ["wp1", "wp2"]
    wp1: [1.0, 1.0]
    wp2: [-1.0, -1,0]
```

El robot debe ir en orden la coordenada (x, y) de cada uno de ellos, emitiendo un sonido cuando lo considera alcanzado. Se cronometrará el tiempo que tarda en hacerlo.

La velocidad lineal no podrá ser nunca superior a 0.4. Se descalificará a quien incumpla esta regla.

Habrá dos rondas:

- Ronda 1: Habrá 4 waypoints, y ninguno en la posición de un obstáculo.
- Ronda 2: Habrá 3-7 waypoints, alguno de ellos en la posición de un obstáculo. En este caso, se podrá ir al siguiente en cuanto se detecte este caso.

## Resolución
## Behaviour Tree

![image](https://user-images.githubusercontent.com/60138852/157194430-83111665-e6d4-4f97-97ab-3bc545c21f34.png)

##Obstacle avoidance

Nuestra aproximación para evitar los puntos con obstáculos pensamos primeramente en comprobar si el punto estaba en un obstáculo subscribiéndonos a global_costmap, comprobamos que en simulador con tiago funcionaba sin problema, pero el kobuki parece que no publica este costmap, para sobrepasar este problema, pasamos a poner un timeout desde que se manda el goal, si se vence el tiempo pasa al siguiente punto.

Utilizando las funciones de la clase BTActionNode:
```
void Move::on_wait_for_result()
{
  auto elapsed = node_->now() - start_time_;

  if (elapsed < 2s) {
    std::cout << "hi" << std::endl;
  } else {
    std::cout << "FINISH" << std::endl;
    halt();
  }
}
```
