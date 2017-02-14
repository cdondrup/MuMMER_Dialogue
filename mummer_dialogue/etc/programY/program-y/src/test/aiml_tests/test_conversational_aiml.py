import unittest
import os

from test.aiml_tests.client import TestClient
from programy.config import BrainFileConfiguration

class ConversationalTestClient(TestClient):

    def __init__(self):
        TestClient.__init__(self)

    def load_configuration(self, arguments):
        super(ConversationalTestClient, self).load_configuration(arguments)
        self.configuration.brain_configuration._aiml_files = BrainFileConfiguration(os.path.dirname(__file__)+"/../aiml_tests/test_files/conversations", ".aiml", False)

class ConversationalAIMLTests(unittest.TestCase):

    def setUp(self):
        ConversationalAIMLTests.test_client = ConversationalTestClient()

    def test_basic_conversational(self):
        response = ConversationalAIMLTests.test_client.bot.ask_question("test",  "HELLO")
        self.assertEqual(response, 'HELLO, WORLD')

        response = ConversationalAIMLTests.test_client.bot.ask_question("test", "GOODBYE")
        self.assertEqual(response, 'SEE YA')

