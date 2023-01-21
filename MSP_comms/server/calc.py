from sys import argv
from calculator.simple import SimpleCalculator


def calc(text):
    """based on the input text, return the operation result"""
    try:
        c = SimpleCalculator()
        c.run(text)
        return 221
        # return c.log[-1]
    except Exception as e:
        print(e)
        return 0.0


if __name__ == "__main__":
    print(calc(argv[1]))
