#include "capstone.hpp"

int special_key_flag = NOT_A_SPECIAL_KEY;

void IO::setPrompt(void (*const prompt)(const char *msg))
{
    this->prompt = prompt;
    this->clear();
}

bool IO::runPrompt()
{
    char *msg = NULL;
    int ch = '\0';
    bool prompt_routine_breaked = false;

    ch = getc();
    prompt_routine_breaked = takech(ch);
    if (prompt_routine_breaked) {
        sync(msg);
        prompt(msg);
        clear();
        return true;
    }
    else {
        return false;
    }
}

int IO::getc()
{
    if (pc.readable()) {
        const int ch = pc.getc();
        switch (ch) {
        case 0:
        case 224:
            special_key_flag = ch;
            return pc.getc();
        default:
            special_key_flag = NOT_A_SPECIAL_KEY;
            return ch;
        }
    }
    else {
        special_key_flag = NOT_A_SPECIAL_KEY;
        return 0;
    }
}

void IO::clear()
{
    for (std::size_t i = 0; i < len(buffer); i++) {
        buffer[i] = '\0';
    }
    cursor = 0;
    theend = 0;
}

bool IO::takech(const int ch)
{
    switch (special_key_flag) {
    case 0:
        break;
    case 224:
        switch (ch) {
        case DEL_KEY:
            if (theend > cursor)
                buffer[--theend] = '\0';
            print();
            break;
        case LEFT_DIRECTION:
            if (cursor > 0) {
                cursor--;
            }
            print();
            break;
        case RIGHT_DIRECTION:
            if (theend >= len(buffer))
                break;
            if (cursor < theend) {
                cursor++;
            }
            print();
            break;
        }
        break;
    case NOT_A_SPECIAL_KEY:
        switch (ch) {
        default:
            if (cursor < 0)
                break;
            if (cursor > theend)
                break;
            if (theend + 1 >= len(buffer))
                break;
            for (int i = theend; i >= cursor; i--) {
                buffer[i + 1] = buffer[i];
            }
            buffer[cursor++] = ch;
            buffer[++theend] = '\0';
            print();
            break;
        case '\b':
            if (cursor > theend)
                break;
            if (theend >= len(buffer)) {
                if (cursor > 0)
                    cursor--;
                buffer[theend--] = '\0';
                break;
            }
            if (cursor <= 0)
                break;
            cursor--;
            for (int i = cursor; i < theend; i++) {
                buffer[i] = buffer[i + 1];
            }
            if (theend > 0) {
                buffer[theend--] = '\0';
            }
            print();
            break;
        case '\n':
        case '\r':
            result = buffer;
            return true;
        case '\0':
            break;
        case ESC:
            clear();
            printf("\r\n");
            result = NULL;
            return true;
        }
        break;
    }
    result = NULL;
    return false;
}

void IO::sync(char *&msg) const
{
    msg = result;
}

void IO::print() const
{
    int i = 0;
    printf("\r");
    for (i = 0; i < len(buffer); i++)
        printf(" ");
    printf("\r");
    for (i = 0; i < cursor; i++)
        printf(" ");
    for (i = cursor; i < theend; i++)
        printf("%c", buffer[i]);
    printf("\r");
    for (i = 0; i < cursor; i++)
        printf("%c", buffer[i]);
    fflush(stdout);
}
