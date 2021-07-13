class Logger:
    def __init__(self, tag):
        self._tag = tag

    def __call__(self, message):
        print('{}: {}'.format(self._tag, message))