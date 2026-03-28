# main.py
from hackarena3 import run_bot
from wrappers.python.user.src.bot.__main__ import ExampleBot

if __name__ == "__main__":
    # Dodajemy nawiasy (), żeby stworzyć obiekt (instancję)
    bot_instance = ExampleBot() 
    run_bot(bot_instance)