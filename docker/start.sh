CUSER_ID="1000"
CONTAINER_USER="cuser"

IMAGE_NAME=ros2_rust

# Check if there are any containers created from the image
CONTAINERS=$(docker ps -a --filter "ancestor=$IMAGE_NAME" -q)

if [ -n "$CONTAINERS" ]; then
  echo "Image has been used to start a container. Removing it."
  docker stop $IMAGE_NAME
  docker rm -f $IMAGE_NAME
else
  echo "Image has not been started. Skipping removal."
fi

echo -e "Starting up ros2_rust container \n >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"

docker run -it --privileged \
    --user=${CUSER_ID}:${CUSER_ID}\
    --group-add sudo \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --workdir="/home/${CONTAINER_USER}/workspace" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --net=host \
    --cap-add=sys_nice \
    --name=$IMAGE_NAME \
    $IMAGE_NAME \