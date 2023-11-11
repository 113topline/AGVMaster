Essa pasta é realacionada ao código presente no módulo central. O NodeMCU deve ser conectado num Arduino UNO/MEGA em bypass paar passar os dados pela porta serial do Arduino, seguindo as seguintes ligações.

NodeMCU    -    Arduino UNO/MEGA
  D1       -          TX
  D2       -          RX

Para realizar o bypass, conectar a porta RESET do Arduino no GND.
Após isso, verificar a porta serial do Arduino no Gerenciador de Dispositivos e atribuí-la nas configurações do BluePlant.

Esse código somente pode ser compilado através do PlatformIO, pela IDE VSCode.
