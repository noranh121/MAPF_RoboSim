import unittest
from TestTurtleController import TestControllerNode
from TestBackendEngine import TestBackendEngine
from TestMapParser import TestMapParser


def suite():
    test_suite = unittest.TestSuite()
    test_suite.addTest(unittest.makeSuite(TestControllerNode))
    test_suite.addTest(unittest.makeSuite(TestBackendEngine))
    test_suite.addTest(unittest.makeSuite(TestMapParser))
    return test_suite

if __name__ == '__main__':
    runner = unittest.TextTestRunner(verbosity=2)
    runner.run(suite())
