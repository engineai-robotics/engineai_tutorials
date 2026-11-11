# CPU-only ROS 2: no CUDA/L4T or NVIDIA runtime dependency on the PM01.
ARG ROS_BASE=ros:humble-ros-base-jammy@sha256:d9bcf2e6f72df127fdee07048c2e87b1b4b9a59a74af0592f4489fc2844aa855
FROM ${ROS_BASE} AS dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
COPY docker/hardware/packages.txt /tmp/pm01-packages.txt
COPY scripts/config/nav2_packages.txt /tmp/pm01-nav2-packages.txt
RUN cat /tmp/pm01-nav2-packages.txt >> /tmp/pm01-packages.txt \
    && apt-get update && xargs -a /tmp/pm01-packages.txt apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/* /tmp/pm01-packages.txt
FROM dependencies AS build
WORKDIR /opt/pm01_navigation
COPY scripts/ scripts/
COPY src/pm01_navigation_runtime/ src/pm01_navigation_runtime/
COPY .deps/hardware/livox_sdk2/ .deps/hardware/livox_sdk2/
ARG BUILD_JOBS=2
RUN python3 scripts/fetch_hardware_dependencies.py --verify-only \
    && PM01_BUILD_JOBS=${BUILD_JOBS} bash scripts/build_sdk.sh \
       /opt/pm01_navigation/.deps/hardware/livox_sdk2 /opt/pm01_navigation/.deps/prefix
COPY scripts/build.sh scripts/
COPY src/livox_ros_driver2/ src/livox_ros_driver2/
RUN PM01_BUILD_JOBS=${BUILD_JOBS} bash scripts/build.sh hardware driver
COPY src/FAST_LIO_ROS2/ src/FAST_LIO_ROS2/
RUN PM01_BUILD_JOBS=${BUILD_JOBS} bash scripts/build.sh hardware lio
COPY src/ src/
COPY config/ config/
RUN PM01_BUILD_JOBS=${BUILD_JOBS} bash scripts/build.sh hardware navigation \
    && rm -rf build log .deps/sdk-build

FROM dependencies AS runtime
WORKDIR /opt/pm01_navigation
COPY --from=build /opt/pm01_navigation /opt/pm01_navigation
COPY scripts/ scripts/
COPY docker/ docker/
COPY LICENSE THIRD_PARTY.md dependencies.repos ./
ARG SOURCE_REVISION=unknown
ARG SOURCE_SHA256=unknown
LABEL org.opencontainers.image.title="PM01 FAST-LIO Navigation" \
      org.opencontainers.image.version="1.0.0" \
      org.opencontainers.image.revision="${SOURCE_REVISION}" \
      cn.engineai.pm01.project="fastlio_navigation" \
      cn.engineai.pm01.profile="hardware" \
      cn.engineai.pm01.source-sha256="${SOURCE_SHA256}"
RUN dpkg-query -W -f='${Package}\t${Version}\t${Architecture}\n' > container-packages.tsv \
    && printf '%s\nsource_sha256=%s\n' "${SOURCE_REVISION}" "${SOURCE_SHA256}" > container-revision.txt
COPY docker/hardware/entrypoint.sh /usr/local/bin/pm01-env
RUN chmod 755 /usr/local/bin/pm01-env
ENTRYPOINT ["/usr/local/bin/pm01-env"]
CMD ["/usr/bin/tini", "--", "sleep", "infinity"]
