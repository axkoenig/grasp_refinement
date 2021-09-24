import os
import csv
import rospy


class Writer:
    def __init__(self, hparams):
        self.hparams = hparams

    def write(self, io_buffer, log_suffix):
        # this happens when we reset the first time
        if len(io_buffer) == 0:
            return

        # get header information
        header = [*io_buffer[0]]

        rospy.loginfo("Writing episode io to disk ...")
        save_path = os.path.join(self.hparams["log_path"], self.hparams["log_name"] + "_" + log_suffix + "_io.csv")
        with open(save_path, "a") as file:
            writer = csv.DictWriter(file, fieldnames=header)
            writer.writeheader()
            writer.writerows(io_buffer)
            file.close()

        rospy.loginfo("Done!")
