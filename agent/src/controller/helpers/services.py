import rospy

# some Gazebo services take strings as requests instead of request objects (hence creating an request object to make compatible with other code)
class StringServiceRequest:
    def __init__(self, part_name, reference_frame):
        self.part_name = part_name
        self.reference_frame = reference_frame


class GazeboServiceException(Exception):
    pass


def service_call(service, request, service_name):
    rospy.wait_for_service(service_name)
    try:
        if request.__class__.__name__ == "StringServiceRequest":
            res = service(request.part_name, request.reference_frame)
            return True, res
        else:
            res = service(request) if request else service()
            return True, res
    except rospy.ServiceException as e:
        rospy.logwarn(f"Service call to {service_name} failed with exception: {str(e)}")
        return False, None


def service_call_with_retries(service, request=None, max_retries=100, wait_secs=0.1):
    tries = 0
    service_name = service.protocol.resolved_name
    while tries < max_retries:
        no_exception, res = service_call(service, request, service_name)
        try:
            success = res.success
        except AttributeError:
            # result did not have attribute "success", we assume everything's ok (e.g. it may be of type EmptyResponse)
            success = True
        if no_exception and success:
            return res
        rospy.loginfo(f"Service call to {service_name} failed. Trying again in {wait_secs} secs...")
        tries += 1
        rospy.sleep(wait_secs)
    raise GazeboServiceException(f"Service call to {service_name} failed even after {max_retries} retries.")


def ros_vector_to_list(ros_vector):
    return [ros_vector.x, ros_vector.y, ros_vector.z]
