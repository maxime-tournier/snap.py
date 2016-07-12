
def coroutine(f):
    '''coroutine decorator.
    
    automatically bootstraps coroutines by advancing them once after
    creation.
    '''

    def start(*args, **kwargs):
        res = f(*args, **kwargs)
        next(res)

        return res

    return start
