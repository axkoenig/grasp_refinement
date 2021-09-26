FROM axkoenig/reflex_stack

# init catkin workspace, copy Reflex Stack over and build it
ENV CATKIN_WS=/home/catkin_ws
COPY agent ${CATKIN_WS}/src/agent
COPY trained_agents ${CATKIN_WS}/src/trained_agents
COPY requirements.txt ${CATKIN_WS}/src/
WORKDIR ${CATKIN_WS}
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# install pip3 package
RUN apt -y update && apt -y upgrade 
RUN apt install -y python3-pip 

# install pip requirements
RUN pip3 install -r ${CATKIN_WS}/src/requirements.txt 

# train agent
CMD ["/bin/bash", "-c", "source ${CATKIN_WS}/devel/setup.bash; python3 ${CATKIN_WS}/src/agent/src/main.py" ]