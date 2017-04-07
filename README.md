# MuMMER Dialogue
This is the dialogue module for MuMMER, which fires up once Pepper reaches the selected to interact user. It follows the same behaviour as in [this video](https://www.youtube.com/watch?v=hKjLpCEzzX8) (currently), combining a chotbot with an MDP learned policy for action selection.

For installtion and usage instructions, please visit: http://protolab.aldebaran.com:9000/mummer/documentations/wikis/wp4-demo-system and http://protolab.aldebaran.com:9000/mummer/documentations/wikis/wp4_dialogue_usage

### Structure:
- **action:** The ros action for invoking the dialogue
- **scripts:** Main script of the module. Entry point
- **src:** Python objects for *State*, *Shop List object*, etc
- **etc:** Additional files like the MDP policy (jsonDump.json), the chatbot files, etc

The dialogue uses explicit turntaking between Pepper and **one** user where Pepper is listening only during the user's turn. After the user stops talking, an action is selected based on the pre-trained policy, after checking the current state (altered by what the user did in the previous turn). If a chat action is decided, the last user utterance is forwarded to the chatbot to be handled.

Currently there are only three sample shop categories: coffee shop, clothing shop, electronics shop.

### Actions
The action space is as follows:
- Greet
- Chat  (the chatbot is fired)
- Task Consume (Pepper enumerates nearby shops according to what the user asked, take selfie or hand out voucher)
- Give Directions (Pepper gives direction to a specific shop, using gestures and spoken language. The spoken directions are hardcoded in the database for each shop)
- Wait (Listens to the user)
- Confirm (If ASR confidence is low, confirm what the user said) [**under development**]
- Request Shop (tied with the voucher distribution action, in case the user did not specify a shop)
- Request Task (one of the actions fired to try to re-engage, if the user stays silent for too long or moved away from the robot without saying goodbye first (meaning they got most probably bored or frustrated))
- Goodbye (Disnegage the current user and goes back to "home" location)
 
### Examples and sample utterances
Some example (task-based) commands include:
- *"I want to buy [coffee/capuccino/trousers/iPhone/shoes]"* (etc)
- *"Are there any shops that have sales"*
- *"I would like a voucher for [Tesco]"*
- *"How can I go to [Starbucks]"*
- *"Let's take a picture"*
 
### NAOqi Concepts
```
'concept:(bye) [bye Goodbye farewell "see you later"]\n'
'concept:(coffee) [coffee cappuccino latte espresso americano]\n'
'concept:(shop) [starbucks costa public "hardware electronics" tesco primark "phone heaven"]\n'
'concept:(electronics) [iPhone Samsung case adapter television TV charger mobile phone]\n'
'concept:(clothing) [shoes jacket "t-shirt" belt jeans trousers shirt coat underwear clothing]\n'
'concept:(selfie) [selfie picture photo photograph]\n'
'concept:(voucher) [voucher sale sales bargain bargains "special offers"]\n'
```
