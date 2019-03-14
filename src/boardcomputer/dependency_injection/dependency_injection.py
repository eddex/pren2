class FeatureBroker:
    def __init__(self, allow_replace=False):
        self.providers = {}
        self.allowReplace = allow_replace

    def provide(self, feature, provider, *args, **kwargs):
        if not self.allowReplace:
            assert not self.providers.keys().__contains__(feature), "Duplicate feature: %r" % feature
        if callable(provider):

            def call():
                return provider(*args, **kwargs)
        else:

            def call():
                return provider

        self.providers[feature] = call

    def __getitem__(self, feature):
        try:
            provider = self.providers[feature]
        except KeyError:
            raise KeyError("Unknown feature named %r" % feature)
        return provider()


features = FeatureBroker()

# Some basic assertions to test the suitability of injected features


def no_assertion(obj):
    return True


def is_instance_of(*classes):
    def test(obj):
        return isinstance(obj, classes)

    return test


def has_attributes(*attributes):
    def test(obj):
        for each in attributes:
            if not hasattr(obj, each):
                return False
        return True

    return test


def has_methods(*methods):
    def test(obj):
        for each in methods:
            try:
                attr = getattr(obj, each)
            except AttributeError:
                return False
            if not callable(attr): return False
        return True

    return test


class RequiredFeature(object):
    def __init__(self, feature, assertion=no_assertion):
        self.feature = feature
        self.assertion = assertion

    def __get__(self, obj, t):
        return self.result  # <-- will request the feature upon first call

    def __getattr__(self, name):
        assert name == 'result', "Unexpected attribute request other then 'result'"
        self.result = self.request()
        return self.result

    def request(self):
        obj = features[self.feature]
        assert self.assertion(obj), \
            "The value %r of %r does not match the specified criteria" \
            % (obj, self.feature)
        return obj
