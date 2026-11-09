# Build and test environment of the interface package: ROS 2 Lyrical Luth (the newest LTS, Ubuntu 26.04, supported to 2031), pinned by
# digest. Nothing of the repository is copied into the image: the source is mounted at run time and built into a mounted cache.
FROM ros:lyrical-ros-base-resolute@sha256:3224ee150cb02eb89a5e0811bc267b57ed3a91b9c10191df57810726a7e916d7

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-lyrical-geographic-msgs \
        ros-lyrical-ament-cmake-pytest \
        python3-colcon-common-extensions \
        python3-pytest python3-yaml \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
