from collections import namedtuple

from duckietown_utils import logger
from .list_of_checks import get_checks
from what_the_duck.check import CheckError, CheckFailed
from duckietown_utils.system_cmd_imp import indent


FAIL = 'check failed'
ERROR = '*check error*'
OK = 'check passed'
SKIP = 'skipped'
statuses = [FAIL, ERROR, OK, SKIP]

Result = namedtuple('Result', 'entry status out_short out_long')

def do_all_checks():
    entries = get_checks()
    results = run_checks(entries)
    display_results(results)
    nfailures = len( [_.status != OK for _ in results] )
    return nfailures
    
def run_checks(entries):
    """ Returns the names of the failures  """
    results = [] 
    
    def record_result(r):
        results.append(r) 
    
    # raise NotRun if not previously run
    class NotRun(Exception): pass
    
    def get_previous_result_status(e):
        for r in results:
            if e == r.entry:
                return r.status
            
        logger.error('Could not find %s' % e)
        logger.error(results)
        raise NotRun()
    
    for entry in entries:
        
        # check dependencies
        only_run_if = entry.only_run_if
        if only_run_if is None:
            pass
        else:
            try:
                dep_status = get_previous_result_status(only_run_if)
            
                if dep_status in [FAIL, ERROR]:
                    msg = "Skipped because dependency %r failed." % (only_run_if.desc)
                    r = Result(entry=entry, status=SKIP, out_short=msg, out_long='')
                    record_result(r)
                    continue
                
                elif dep_status in [SKIP]:
                    msg = "Skipped because dependency %r skipped." % (only_run_if.desc)
                    r = Result(entry=entry, status=SKIP, out_short=msg, out_long='')
                    record_result(r)
                    continue

            except NotRun:
                msg = 'Dependency did not run yet.'
                r = Result(entry=entry, status=ERROR, out_short=msg, out_long='', )
                record_result(r)
                continue
        
        # at this point, either it's None or passed
        assert only_run_if is None or (get_previous_result_status(only_run_if) == OK)
    
        try:
            res = entry.check.check() or ''
            r = Result(entry=entry, status=OK, out_short=res, out_long='')
            record_result(r)
            
        except CheckError as e:
            r = Result(entry=entry, status=ERROR, 
                       out_short='Could not run test.',
                       out_long=str(e))
            record_result(r)
            
        except CheckFailed as e:
            r = Result(entry=entry, status=FAIL, 
                       out_short=e.compact,
                       out_long=e.long_explanation)
            record_result(r)
            
    return results


def display_results(results): 

    for r in results:
        s = '%-30s  %s   %s' % (r.entry.desc, r.status, r.out_short)
        if r.status in [OK]:
            logger.info(s)
        elif r.status in [FAIL]:
            logger.error(s)
            if r.entry.diagnosis is not None:
                s = str(r.entry.diagnosis)
                label = 'diagnosis:  '
                s = indent(s , " "*len(label), label)
                print(s)
#                 logger.warn(str(r.entry.diagnosis))
        elif r.status in [ERROR]:
            logger.error(s)
        elif r.status in [SKIP]:
            logger.debug(s)
        else:
            assert False, r.status
            
    
