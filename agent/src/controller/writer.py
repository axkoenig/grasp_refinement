import os
import csv
import rospy


class Writer:
    def __init__(self, hparams, log_suffix):
        self.hparams = hparams
        self.save_path = os.path.join(hparams["log_path"], hparams["log_name"] + "_" + log_suffix + "_io.csv")
        self.written_header = False

    def write(self, io_buffer):
        # this happens when we reset the first time
        if len(io_buffer) == 0:
            return

        # get header information
        header = [*io_buffer[0]]

        rospy.loginfo("Writing episode io to disk ...")
        with open(self.save_path, "a") as file:
            writer = csv.DictWriter(file, fieldnames=header)
            if not self.written_header:
                writer.writeheader()
                self.written_header = True
            writer.writerows(io_buffer)
            file.close()

        rospy.loginfo("Done!")
