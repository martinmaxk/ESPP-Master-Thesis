using Pathfinding.Internal.TiledUtils;
using System;
using System.Text;

namespace pgraph.grid
{
	// A simple uniform-cost lattice / cell grid.
	// This implementation uses a bitpacked representation
	// in order to improve time and space efficiency.
	//
	// @author: dharabor
	// @created: 2015-04-16
	//

	public class BitpackedGrid
	{
		// a very small number; smaller than the smallest step size
		// provided no individual grid dimension is larger than epsilon^(-1)
		public const double epsilon = 0.0000001;

		// we frame the map with some extra obstacles 
		// this helps us avoid unnecessary boundary testing
		private const int padding_ = 2;

		// we use single int words when packing the grid;
		// each vertex is assigned one bit per word
		private const int BITS_PER_WORD = 32;
		private const int LOG2_BITS_PER_WORD = 5;
		private const int INDEX_MASK = BITS_PER_WORD - 1;

		// the original dimensions of the gridmap
		private int map_height_original_;
		private int map_width_original_;

		// the padded dimensions of the gridmap
		// we round each row to the nearest number of words (+1)
		// and we add one row of padding before the first row
		// one row of padding after the last row
		private int map_height_;
		private int map_width_;
		private int map_size_;
		private int map_width_in_words_; // for convenience

		// data describing the cell grid
		private int[] map_cells_;

		// data relating to the discrete set of lattice points that 
		// together form the cell grid; the data is redundant but 
		// we keep it anyway for better performance.
		private int[] visible_;
		private int[] corner_;
		private int[] double_corner_;

		// there are a finite number of places where an edge can
		// be intersected. This variable stores the smallest 
		// distance between any two (adjacent) such points.
		public double smallest_step;
		public double smallest_step_div2;

		public BitpackedGrid(TileGrid grid)
		{
			this.load(grid);
		}

		// @param width: the horizontal size of the lattice, as 
		// 				 measured in square cells
		// @param height: the vertical size of the lattice, as 
		//				 measured in square cells
		public BitpackedGrid(int width, int height)
		{
			init(width, height);
		}

		private void init(int width, int height)
		{
			this.map_height_original_ = height;
			this.map_width_original_ = width;
			this.map_width_in_words_ = ((width >> LOG2_BITS_PER_WORD) + 1);
			this.map_width_ = map_width_in_words_ << LOG2_BITS_PER_WORD;
			this.map_height_ = height + 2 * padding_;

			this.map_size_ = ((map_height_ * map_width_)
					>> LOG2_BITS_PER_WORD);
			this.map_cells_ = new int[map_size_];
			this.visible_ = new int[map_size_];
			this.corner_ = new int[map_size_];
			this.double_corner_ = new int[map_size_];

			this.smallest_step =
					Math.Min(
						1 / (double)this.get_padded_width(),
						1 / (double)this.get_padded_height());
			this.smallest_step_div2 = smallest_step / 2.0;
		}

		// returns true if the point (x, y) is visible from
		// another discrete point on the grid (i.e. (x, y) is 
		// not adjacent to 4 obstacle tiles).
		public bool get_point_is_visible(int x, int y)
		{
			return get_bit_value(x, y, visible_);
		}

		// returns true if the point (x, y) is adjacent to 
		// (i) exactly one obstacle tile OR (ii) exactly two 
		// diagonally adjacent obstacle tiles
		public bool get_point_is_corner(int x, int y)
		{
			return get_bit_value(x, y, corner_);
		}

		// returns true if the point (x, y) is adjacent to 
		// exactly two diagonally adjacent obstacle tiles.
		public bool get_point_is_double_corner(int x, int y)
		{
			return get_bit_value(x, y, double_corner_);
		}

		// returns true if the cell (x, y) is not an obstacle 
		public bool get_cell_is_traversable(int cx, int cy)
		{
			return get_bit_value(cx, cy, map_cells_);
		}


		public void set_point_is_visible(int x, int y, bool value)
		{
			set_bit_value(x, y, value, visible_);
		}

		public void set_point_is_corner(int x, int y, bool value)
		{
			set_bit_value(x, y, value, corner_);
		}

		public void set_point_is_double_corner(int x, int y, bool value)
		{
			set_bit_value(x, y, value, double_corner_);
		}

		public bool get_point_is_discrete(double x, int y)
		{
			return Math.Abs((int)(x + this.smallest_step_div2) - x)
					< this.smallest_step;
		}

		public void set_cell_is_traversable(int cx, int cy, bool value)
		{
			set_bit_value(cx, cy, value, map_cells_);
			update_point(cx, cy);
			update_point(cx + 1, cy);
			update_point(cx, cy + 1);
			update_point(cx + 1, cy + 1);
		}

		private void update_point(int px, int py)
		{
			bool cellNW = get_cell_is_traversable(px - 1, py - 1);
			bool cellNE = get_cell_is_traversable(px, py - 1);
			bool cellSW = get_cell_is_traversable(px - 1, py);
			bool cellSE = get_cell_is_traversable(px, py);

			bool corner =
					((!cellNW | !cellSE) & cellSW & cellNE) |
					((!cellNE | !cellSW) & cellNW & cellSE);

			bool double_corner =
					((!cellNW & !cellSE) & cellSW & cellNE) ^
					((!cellSW & !cellNE) & cellNW & cellSE);

			bool visible =
					cellNW | cellNE | cellSW | cellSE;

			set_point_is_corner(px, py, corner);
			set_point_is_double_corner(px, py, double_corner);
			set_point_is_visible(px, py, visible);
		}

		// TODO: pass int instead of bool (removes one branch instruction)
		private void set_bit_value(int x, int y, bool value, int[] elts)
		{
			int map_id = get_map_id(x, y);
			int word_index = map_id >> LOG2_BITS_PER_WORD;
			int mask = (1 << ((map_id & INDEX_MASK)));
			int tmp = elts[word_index];
			elts[word_index] = value ? (tmp | mask) : (tmp & ~mask);
		}

		// TODO: pass int instead of bool (removes one branch instruction)
		private bool get_bit_value(int x, int y, int[] elts)
		{
			int map_id = get_map_id(x, y);
			int word_index = map_id >> LOG2_BITS_PER_WORD;
			int mask = 1 << ((map_id & INDEX_MASK));
			return (elts[word_index] & mask) != 0;
		}

		private int get_map_id(int x, int y)
		{
			x += padding_;
			y += padding_;
			return (y * map_width_ + x);
		}

		// Load up map files in MovingAI format
		public void load(TileGrid grid)
		{
			int width = grid.Width;
			int height = grid.Height;
			this.init(width, height);
			for (int y = 0; y < height; y++)
			{
				for (int x = 0; x < width; x++)
				{
					this.set_cell_is_traversable(x, y, grid[x, y] == TileShape.None);
				}
			}
		}

		public string debug_cells(int myx, int myy)
		{
			var buf = new StringBuilder();
			for (int y = 0; y < this.map_height_original_; y++)
			{
				for (int x = 0; x < this.map_width_original_; x++)
				{
					if (myx == x && myy == y)
					{
						buf.Append("X");
					}
					else
					{
						char c = get_cell_is_traversable(x, y) ? '.' : '@';
						buf.Append(c);
					}
				}
				buf.Append("\n");
			}
			return buf.ToString();
		}

		public string print_cells()
		{
			var buf = new StringBuilder();
			for (int y = 0; y < this.map_height_original_; y++)
			{
				for (int x = 0; x < this.map_width_original_; x++)
				{
					char c = get_cell_is_traversable(x, y) ? '.' : '@';
					buf.Append(c);
				}
				buf.Append("\n");
			}
			return buf.ToString();
		}

		public int get_padded_width()
		{
			return this.map_width_;
		}

		public int get_padded_height()
		{
			return this.map_height_;
		}

		public int get_num_cells()
		{
			return this.map_height_ * map_width_;
		}

		// print a portion of the grid cells around location (x, y)
		// @param offset specifies how many cells around (x, y) to print
		// i.e. an offset of 10 prints 21x21 cells with (x,y) in the middle
		// (10 cells above (x, y), 10 below, 10 left and 10 right)
		public void print_cells(int x, int y, int offset, System.IO.TextWriter stream)
		{
			for (int j = y - offset; j < y + offset; j++)
			{
				stream.Write(j + " ");
				for (int i = x - offset; i < x + offset; i++)
				{
					stream.Write(this.get_cell_is_traversable(i, j) ? "." : "@");
				}
				stream.WriteLine("");
			}
		}
	}
}
