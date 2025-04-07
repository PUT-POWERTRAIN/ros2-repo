#!/bin/bash

# Konfiguracja
IMAGE_NAME="powertrain-ros-ubuntu-22"
CONTAINER_NAME="powertrain-ros"
WORKSPACE_DIR="../"  # Ścieżka do Twojego workspace ROS na hoście
DOCKERFILE_PATH="$(dirname "$0")/Dockerfile"  # Zakłada, że Dockerfile jest w tym samym katalogu

# 1. Budowanie obrazu Dockera
echo "🔨 Budowanie obrazu Dockera..."
docker build -t $IMAGE_NAME -f "$DOCKERFILE_PATH" . --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) || {
    echo "❌ Błąd podczas budowania obrazu"; exit 1
}

# 2. Uruchomienie kontenera z GUI i wolumenami
echo "🚀 Uruchamianie kontenera..."
xhost +local:docker  # Zezwól na dostęp do X11

docker run -it --rm --name $CONTAINER_NAME --env="DISPLAY=$DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix" --net=host --volume="$WORKSPACE_DIR:/home/rosuser/ros_ws" --user $(id -u):$(id -g) $IMAGE_NAME /bin/bash

# 3. Sprzątanie
xhost -local:docker  # Zabierz uprawnienia X11
echo "✅ Kontener został zatrzymany"
