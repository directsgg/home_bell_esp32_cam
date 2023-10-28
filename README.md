# Timbre Inteligente con Video y Audio, Home Bell

## Descripción
Este proyecto es un timbre inteligente que utiliza un ESP32CAM, un módulo de micrófono INMP441 y un amplificador de audio MAX98357. El timbre tiene capacidad de video y audio, y se complementa con una aplicación desarrollada en React Native. La carcasa del timbre esta impresa en 3d

## Componentes
1. **ESP32CAM**: Este es el cerebro del timbre inteligente. Se encarga de recibir los datos de la camara integrada, recibir los datos del microfono, establecer un servidor HTTP y un protocolo UDP para la transmision de datos entre el timbre y la app, asi mismo transmitir los datos al amplificador de audio.
2. **Módulo de micrófono INMP441**: Este módulo utiliza una interfaz I2S con multiples caracteristicas como la eliminación de ruido, ganancia entre otros. Se utiliza para obtener el audio analogico, procesarlo y transmitir los datos digitales al microcontrolador
3. **Amplificador de audio MAX98357**: Este amplificador se utiliza para obtener los datos de audio del microcontrolador ESP32 por medio de la interfaz I2S, posteriormente reproducir y amplificar este audio en una bocina

## Código Fuente
El código fuente del proyecto está disponible en este repositorio basado en el microcontrolador esp32cam desarrollado sobre Platform IO.

## Carcasa
La carcasa del timbre inteligente fue creada con una impresora 3D. Los archivos STL para la carcasa están disponibles en esta página.

## Aplicación
La aplicación para android asociada está desarrollada en React Native y permite el control del timbre inteligente tanto de la parte de audio como de video.

## Comentarios
Cualquier duda o comentario estoy a la orden.
