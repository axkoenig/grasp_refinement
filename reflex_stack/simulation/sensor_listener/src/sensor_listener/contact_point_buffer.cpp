#include "sensor_listener/contact_point_buffer.hpp"

SimContactResult ContactPointBuffer::get_averaged_results()
{
    SimContactResult scr;
    int num_contacts = results.size();
    for (int i = 0; i < num_contacts; i++)
    {
        scr.position += results[i].position;
        scr.normal += results[i].normal;
        scr.force += results[i].force;
        scr.torque += results[i].torque;
    }
    scr.position /= num_contacts;
    scr.normal /= num_contacts;
    scr.force /= num_contacts;
    scr.torque /= num_contacts;

    return scr;
}