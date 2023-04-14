import director
import os
import sys
import argparse
import yaml


class DRCArgParser(object):
    def __init__(self):
        self._args = None
        self._parser = None
        self.strict = False

    def getArgs(self):
        if self._args is None:
            self.parseArgs()
        return self._args

    def getParser(self):
        if self._parser is None:
            self._parser = argparse.ArgumentParser()
            self.addDefaultArgs(self._parser)
        return self._parser

    def parseArgs(self):
        parser = self.getParser()
        sys.argv = [str(v) for v in sys.argv]

        if not self.strict:
            self._args, unknown = parser.parse_known_args()
        else:
            self._args = parser.parse_args()

        def flatten(l):
            return [item for sublist in l for item in sublist]

        # now flatten some list of lists
        self._args.data_files = flatten(self._args.data_files)

    def addDefaultArgs(self, parser):
        class IgnoreROSArgsAction(argparse.Action):
            """
            This action allows us to ignore ROS arguments, which are always added and prefixed by __. Also ignores
            remapping arguments, which contain :=
            """

            def __call__(self, parser, namespace, values, option_string=None):
                valid_args = [
                    arg
                    for arg in values
                    if not arg.startswith("__") and ":=" not in arg
                ]
                if getattr(namespace, self.dest):
                    # Already received a config argument previously, need to append to the list
                    getattr(namespace, self.dest).append(valid_args)
                else:
                    setattr(namespace, self.dest, [valid_args])

        parser.add_argument(
            "--robot-config",
            dest="robotConfigs",
            action=IgnoreROSArgsAction,
            nargs="+",
            metavar=("file_path", "robot_name"),
            help="YAML files specifying configurations for robots to display. Can be provided "
            "multiple times to display multiple robots. The second argument specifying the robot "
            "name is required if using the same configuration file more than once.",
        )

        parser.add_argument(
            "--data",
            type=str,
            nargs="+",
            dest="data_files",
            default=[],
            action="append",
            metavar="filename",
            help="data files to load at startup",
        )

        parser.add_argument(
            "--script",
            "--startup",
            type=str,
            nargs="+",
            dest="scripts",
            default=[],
            action="append",
            metavar="filename",
            help="python scripts to run at startup",
        )


_argParser = None


def getGlobalArgParser():
    global _argParser
    if not _argParser:
        _argParser = DRCArgParser()
    return _argParser


def requireStrict():
    global _argParser
    _argParser = None
    getGlobalArgParser().strict = True


def args():
    return getGlobalArgParser().getArgs()


def getRobotConfig(robotName=""):
    return DirectorConfig.getDefaultInstance().getConfig(robotName)
