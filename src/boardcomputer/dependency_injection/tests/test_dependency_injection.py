from dependency_injection.dependency_injection import RequiredFeature, is_instance_of, features


class TestClass:
    name = RequiredFeature('name', is_instance_of(str))

    def get_name(self):
        return self.name


def test_string_resolve():
    expected_resolved_name = 'my resolved name'
    features.provide('name', expected_resolved_name)

    test_object = TestClass()
    assert expected_resolved_name == test_object.get_name()
