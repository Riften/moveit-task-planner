import pddl.functions
import pddl_parser.pddl_file as pddl_file

domain_file = "/home/yongxi/Workspace/ws_moveit2/src/moveit-task-planner/resources/pddl_script/domain-02.pddl"
problem_file = "/home/yongxi/Workspace/ws_moveit2/src/moveit-task-planner/resources/pddl_script/problem-02.pddl"

pddl_task = pddl_file.open(domain_file, problem_file)
print(pddl_task)