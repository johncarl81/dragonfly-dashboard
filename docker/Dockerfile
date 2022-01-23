FROM ros:melodic

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV DEBIAN_FRONTEND noninteractive
ENV WORKSPACE /workspace
ENV MAVEN_VERSION 3.8.4

# Install dependencies
RUN apt-get -qq update && \
    apt-get -qq upgrade && \
    apt-get -qq install apt-utils build-essential vim python-catkin-tools software-properties-common

# Setup Java JDK pacakge repository
RUN add-apt-repository ppa:linuxuprising/java && apt-get -qq update && \
	echo "oracle-java17-installer-local shared/accepted-oracle-license-v1-3 select true" | debconf-set-selections

RUN apt-get -qq install ros-melodic-mavros ros-melodic-mavros-extras oracle-java17-installer oracle-java17-set-default openjfx unzip ros-melodic-rosbridge-server

# Install maven
RUN wget https://downloads.apache.org/maven/maven-3/$MAVEN_VERSION/binaries/apache-maven-${MAVEN_VERSION}-bin.tar.gz -P /tmp
RUN tar xf /tmp/apache-maven-${MAVEN_VERSION}-bin.tar.gz -C /opt
RUN ln -s /opt/apache-maven-$MAVEN_VERSION /opt/maven

RUN export JAVA_HOME
ENV M2_HOME /opt/maven
ENV MAVEN_HOME /opt/maven
ENV PATH "$M2_HOME/bin:$PATH"

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Clear apt cache to save on space
RUN rm -rf /var/lib/apt/lists/*

RUN echo "Xft.dpi: 192" > ~/.Xdefaults

# Setup Gradle - hoping this aids performance
RUN mkdir -p $WORKSPACE/gradle/wrapper
ADD build.gradle $WORKSPACE
ADD gradlew $WORKSPACE
ADD gradle/wrapper/* $WORKSPACE/gradle/wrapper/

RUN cd $WORKSPACE && ./gradlew clean
COPY . $WORKSPACE

# Build and install java_rosbridge submodule
RUN cd $WORKSPACE/java_rosbridge && \
    mvn clean install

# Install dragonfly infrastructure
RUN source /opt/ros/melodic/setup.bash && \
	cd $WORKSPACE/dragonfly && \
	rm -rf src/dragonfly_dashboard src/dragonfly_messages_java build devel && \
	catkin_make && \
	catkin_make install

# Install dragonfly desktop

RUN cd $WORKSPACE && \
	./gradlew build --parallel

# Setup Entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]