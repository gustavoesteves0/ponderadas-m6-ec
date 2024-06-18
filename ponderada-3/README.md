# Turtlebot Teleoperado PT2

## Objetivo
Incrementar o desenvolvimento da atividade anterior para incluir streaming de imagens.

## Atividade Desenvolvida

### Backend
Seguindo o desenvolvimento da atividade anterior, começaremos com a movimentação do turtlebot:

#### Nó de Controle do Robô

A classe `RobotControlNode` estende `rclpy.node.Node` e gerencia os comandos de movimento do turtlebot. Ela inicializa os publicadores e serviços necessários e publica periodicamente comandos de velocidade para o robô.

**Componentes principais:**

- **Publicador de Velocidade**: Publica mensagens `Twist` no tópico `cmd_vel` para controlar as velocidades linear e angular do robô.
- **Serviços:**
    - `stop_robot_service`: Para o turtlebot imediatamente.
    - `shutdown_robot_service`: Desliga o robô e o nó ROS.

#### Métodos

- `__init__()`: Inicializa o nó, publicador e serviços. Configura um callback de temporizador para publicar comandos de velocidade.
- `stop_robot_service(request, response)`: Lida com a chamada do serviço de parada do robô.
- `shutdown_robot_service(request, response)`: Lida com a chamada do serviço de desligamento do turtlebot.
- `stop_robot()`: Para o robô, definindo as velocidades como zero.
- `shutdown_robot()`: Desliga o turtlebot e o ROS 2.
- `set_velocity(linear, angular)`: Define as velocidades linear e angular para o robô.
- `timer_callback()`: Publica o comando de velocidade atual se o turtlebot estiver em execução.

#### Serviços e Publicadores ROS 2

- **Publicador**:
  - `self.velocity_pub`: Publica comandos de velocidade no tópico `cmd_vel`.
- **Serviços**:
  - `self.stop_srv`: Lida com solicitações para parar o robô.
  - `self.shutdown_srv`: Lida com solicitações para desligar o turtlebot.

#### Endpoints de Controle do Turtlebot

A aplicação FastAPI fornece vários endpoints para controlar o robô:

- `GET /forward`: Move o robô para frente.
- `GET /backward`: Move o robô para trás.
- `GET /left`: Vira o turtlebot para a esquerda.
- `GET /right`: Vira o robô para a direita.
- `GET /stop`: Para o turtlebot.
- `GET /shutdown`: Desliga o robô.

Cada endpoint define as velocidades apropriadas chamando o método `set_velocity()` ou para o robô chamando o método `stop_robot()`.

### Streaming

#### Endpoint de Streaming de Vídeo

A aplicação inclui um endpoint para transmitir vídeo de uma câmera conectada usando OpenCV:

- `GET /stream`: Transmite quadros de vídeo capturados da câmera.

**Método**

- `generate_video()`: Captura quadros da câmera e os codifica como imagens JPEG. Cada quadro é então enviado como parte de uma resposta HTTP multipart, permitindo a transmissão de vídeo em tempo real. O método também calcula e inclui a latência de captura de cada quadro.

#### Medição de Latência

A aplicação fornece um endpoint para estimar a latência de captura de um quadro da câmera:

- `GET /latency`: Mede o tempo necessário para capturar um único quadro e retorna a latência como uma resposta JSON.

**Método**

- `estimate_latency()`: Captura um único quadro da câmera e calcula o tempo necessário para isso. O valor da latência é então retornado em milissegundos.

### Index.html
Para receber o streaming de imagens, foi adicionado um arquivo HTML, que exibe a transmissão. Aqui também estão os botões de controle do robô, que enviam os comandos de movimentação para o backend, fazendo com que ele se mova.

#### Descrição dos Componentes

- Botões de Controle: Botões que enviam comandos para mover o robô em diferentes direções ou parar/desligar o robô.
- Informações de Latência: Exibe a latência estimada da captura de vídeo.
- Streaming de Vídeo: Exibe a transmissão de vídeo ao vivo da câmera conectada ao robô.

#### Funções JavaScript
- `sendCommand(command)`: Envia uma solicitação de comando para o backend usando a função `fetch`
- `updateLatency()`: Atualiza a latência estimada periodicamente (a cada segundo) solicitando a latência do backend e exibindo-a na página.

### Utilização da Solução

Para executar a aplicação, é necessário seguir os seguintes passos:

1. Entrar na raíz do projeto:

```
cd ponderada-3/
```

2. Ligar o venv e instalar os requirements do projeto:

```
source venv/bin/activate
python3 pip install -r requirements.txt
```

3. Executar o código da aplicação:

```
python3 main.py
```
Após a execução desses passos, já é possível acessar a interaface no localhost http://0.0.0.0:8000/. 

4. Num segundo terminal, o usuário deve rodar o webots, para que seja possível simular a movimentação do turtlebot3. Para isso, deve-se executar o comando:

```
ros2 launch webots_ros2_turtlebot robot_launch.py
```

### Vídeo
O vídeo abaixo serve de demonstração da utilização da solução.

[![V[ideo de demonstração da solução]](https://img.youtube.com/vi/KCeaGp_yR9Y/0.jpg)](https://youtu.be/KCeaGp_yR9Y)