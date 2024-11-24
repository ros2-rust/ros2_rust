echo -e "Building ros2_rust:lastest image"
name=ros2_rust
DOCKER_BUILDKIT=1 \
docker build --pull --rm -f ../Dockerfile \
--build-arg BUILDKIT_INLINE_CACHE=1 \
--tag $name:latest .