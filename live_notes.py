import datetime
import argparse

def save_text(text, filename):
    timestamp = datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S]")

    with open(filename, "a") as file:
        file.write(f"{timestamp}--> {text}\n")

    # print("Text saved.")

def main(filename):

    # filename = "text_entries.txt"
    print("Text Entry Program")
    print("Enter 'quit' to exit.")

    interventions = 0

    while True:
        user_input = input("Enter text: ")

        if user_input == "q":
            break
        if user_input == "1":
            user_input = "UNDESIRABLE BEHAVIOR"
            interventions += 1

        save_text(user_input, filename)

    with open(filename, 'r+') as fp:
        lines = fp.readlines()
        lines.insert(0, "Number of Undesriable Behavior: " + str(interventions) + "\n")
        fp.seek(0)
        fp.writelines(lines)




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Text Entry Program")
    parser.add_argument("-f", "--filename", type=str, default="text_entries.txt", help="Name of the output file (default: text_entries.txt)")
    args = parser.parse_args()

    main(args.filename)
