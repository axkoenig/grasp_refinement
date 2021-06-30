import os


class Writer:
    def __init__(self, hparams):
        self.hparams = hparams
        save_path = os.path.join(hparams["log_path"], hparams["log_name"] + "_io.csv")

    def write(self, obs_dict, action_dict, reward, infos):
        # concat into one dict
        super_dict = {}
        dicts = [obs_dict, action_dict, {"reward": reward}, infos]
        for d in dicts:
            for k, v in d.iteritems():
                super_dict.setdefault(k, []).append(v)
        
        