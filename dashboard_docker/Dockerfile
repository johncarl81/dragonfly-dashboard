FROM ros:melodic

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV DEBIAN_FRONTEND noninteractive
ENV WORKSPACE=/workspace

# Install dependencies
RUN apt-get -qq update && \
    apt-get -qq upgrade && \
    apt-get -qq install apt-utils build-essential vim python-catkin-tools software-properties-common mesa-utils

# Install Java
RUN add-apt-repository ppa:linuxuprising/java && apt-get -qq update && \
	echo "oracle-java11-installer-local shared/accepted-oracle-license-v1-2 select true" | debconf-set-selections

RUN mkdir -p /var/cache/oracle-jdk11-installer-local/
COPY dashboard_docker/jdk-11.0.9_linux-x64_bin.tar.gz /var/cache/oracle-jdk11-installer-local/

RUN apt-get -qq install oracle-java11-installer-local python-pip openjfx unzip

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Clear apt cache to save on space
RUN rm -rf /var/lib/apt/lists/*

# Setup arcgis workspace
ENV ARCGIS_VERSION=100.8.0
RUN mkdir -p /root/.arcgis/$ARCGIS_VERSION/
COPY ./dashboard_docker/arcgis-java-jnilibs-$ARCGIS_VERSION.zip /tmp/

RUN unzip /tmp/arcgis-java-jnilibs-$ARCGIS_VERSION.zip -d /root/.arcgis/$ARCGIS_VERSION/ \
    && rm /tmp/arcgis-java-jnilibs-$ARCGIS_VERSION.zip

COPY ./dashboard_docker/arcgis-java-resources-$ARCGIS_VERSION.zip /tmp/

RUN unzip /tmp/arcgis-java-resources-$ARCGIS_VERSION.zip -d /root/.arcgis/$ARCGIS_VERSION/ \
	&& rm /tmp/arcgis-java-resources-$ARCGIS_VERSION.zip

# Install dragonfly desktop
RUN mkdir -p $WORKSPACE
COPY . $WORKSPACE

RUN cd $WORKSPACE && ./gradlew clean build

# Setup Entrypoint
COPY ./dashboard_docker/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]