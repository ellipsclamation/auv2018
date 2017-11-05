import cmd

from modules import example

class auv(cmd.Cmd):
	intro = '\nType help or ? to list commands.'
	prompt = 'auv> '

	def do_test(self, arg):
		'\nthis is documentation'
		print('test')
		print(arg)

	def do_example(self, arg):
		'\nCalls the example module\'s print_example function'
		example.print_example(arg)

	def do_exit(self, arg):
		'\nExits auv'
		return True

if __name__ == '__main__':
	auv().cmdloop()