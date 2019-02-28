from dummycomponent.dummy import Dummy


def test_add():
    assert 3 == Dummy().add(1, 2)
