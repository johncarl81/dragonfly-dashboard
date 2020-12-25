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
    apt-get -qq install apt-utils build-essential vim python-catkin-tools software-properties-common

# Install Java
RUN add-apt-repository ppa:linuxuprising/java && apt-get -qq update && \
	echo "oracle-java11-installer-local shared/accepted-oracle-license-v1-2 select true" | debconf-set-selections

RUN mkdir -p /var/cache/oracle-jdk11-installer-local/
COPY docker/jdk-11.0.9_linux-x64_bin.tar.gz /var/cache/oracle-jdk11-installer-local/

RUN apt-get -qq install ros-melodic-mavros ros-melodic-mavros-extras oracle-java11-installer-local openjfx unzip

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Install ROS Bridge
RUN apt-get -qq install ros-melodic-rosbridge-server

# Clear apt cache to save on space
RUN rm -rf /var/lib/apt/lists/*

# Setup arcgis workspace
ENV ARCGIS_VERSION=100.9.0
RUN mkdir -p /root/.arcgis/$ARCGIS_VERSION/
COPY ./docker/arcgis-java-jnilibs-$ARCGIS_VERSION.zip /tmp/

RUN unzip /tmp/arcgis-java-jnilibs-$ARCGIS_VERSION.zip -d /root/.arcgis/$ARCGIS_VERSION/ \
    && rm /tmp/arcgis-java-jnilibs-$ARCGIS_VERSION.zip

COPY ./docker/arcgis-java-resources-$ARCGIS_VERSION.zip /tmp/

RUN unzip /tmp/arcgis-java-resources-$ARCGIS_VERSION.zip -d /root/.arcgis/$ARCGIS_VERSION/ \
	&& rm /tmp/arcgis-java-resources-$ARCGIS_VERSION.zip

# Install java_rosbridge snapshot
ENV JAVA_ROSBRIDGE_DIR=edu/brown/cs/burlap/java_rosbridge
ENV JAVA_ROSBRIDGE_VERSION=2.0.3-SNAPSHOT
RUN mkdir -p /root/.m2/repository/$JAVA_ROSBRIDGE_DIR/$JAVA_ROSBRIDGE_VERSION/
COPY ./docker/*.jar /root/.m2/repository/$JAVA_ROSBRIDGE_DIR/$JAVA_ROSBRIDGE_VERSION/
COPY ./docker/*.pom /root/.m2/repository/$JAVA_ROSBRIDGE_DIR/$JAVA_ROSBRIDGE_VERSION/


RUN mkdir -p $WORKSPACE
COPY . $WORKSPACE
# Install dragonfly infrastructure
RUN source /opt/ros/melodic/setup.bash && \
	cd $WORKSPACE/dragonfly && \
	rm -rf src/dragonfly_dashboard && \
	rm -rf src/dragonfly_messages_java && \
	catkin_make && \
	catkin_make install

# Install dragonfly desktop

RUN cd $WORKSPACE && \
	./gradlew build --parallel

# Setup Entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]