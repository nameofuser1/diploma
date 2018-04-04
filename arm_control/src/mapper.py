from ABC import abstractmethod, abstractproperty


class XacroCommander(object):

    def __init__(self):
        pass

    def find(self, name):
        return "$(find " + name + ")"


class MapObject(object):

    def __init__(self):
        self._commander = XacroCommander()

    @abstractproperty
    def mesh(self):
        raise NotImplementedError()


class Cell(MapObject):

    def __init__(self):
        super(MapObject, self).__init__()

    @property
    def mesh(self):
        return self._commander.find() + "/"

