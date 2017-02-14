import unittest
import os
from test.aiml_tests.client import TestClient
from programy.config import BrainFileConfiguration

class BasicTestClient(TestClient):

    def __init__(self):
        TestClient.__init__(self)

    def load_configuration(self, arguments):
        super(BasicTestClient, self).load_configuration(arguments)
        self.configuration.brain_configuration._aiml_files = BrainFileConfiguration(os.path.dirname(__file__)+"/../aiml_tests/test_files/underline_star", ".aiml", False)

class UnderlineStarAIMLTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        UnderlineStarAIMLTests.test_client = BasicTestClient()

    def test_underline_first(self):
        response = UnderlineStarAIMLTests.test_client.bot.ask_question("test",  "SAY HEY")
        self.assertIsNotNone(response)
        self.assertEqual(response, 'UNDERLINE IS SAY')

    def test_underline_last(self):
        response = UnderlineStarAIMLTests.test_client.bot.ask_question("test", "HELLO THERE")
        self.assertIsNotNone(response)
        self.assertEqual(response, 'UNDERLINE IS THERE')

    def test_underline_middle(self):
        response = UnderlineStarAIMLTests.test_client.bot.ask_question("test", "HI KEIFF MATE")
        self.assertIsNotNone(response)
        self.assertEqual(response, 'UNDERLINE IS KEIFF')
