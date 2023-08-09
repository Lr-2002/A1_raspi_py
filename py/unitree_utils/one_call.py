import functools
def only_run_once(func):
    @functools.wraps(func)
    def wrapper_only_run_once(*args, **kwargs):
        if wrapper_only_run_once.first_call == 0:
            return wrapper_only_run_once.result
        else:
            wrapper_only_run_once.first_call = 0
            wrapper_only_run_once.result = func(*args, **kwargs)
            return wrapper_only_run_once.result
    wrapper_only_run_once.first_call = 1
    wrapper_only_run_once.result=0
    return wrapper_only_run_once

@only_run_once
def get_max_age():
    print('here i am')
    return 2
if __name__ == '__main__':
    print(get_max_age())
    print(get_max_age())
